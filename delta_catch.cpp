#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <memory>
#include <iomanip>

#include <vision/nxtcam.h>

#include "debug.h"
#include "delta_hand.h"
#include "trajectory.h"

#include "stereo_config.h"
#include "delta_robot_args.h"

#include "viz_client.h"

using namespace ev3dev;
using namespace std;

static cl::arg<vector3i> pid(
  vector3i(5000, 5000, 100),
  cl::name("pid"),
  cl::desc("PID parameters Kp, Ki, Kd."));

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

static cl::arg<float> max_flight_time(
  1.25f,
  cl::name("max-flight-time"),
  cl::desc("The longest time allowed for a single trajectory."));

static cl::arg<std::string> hand(
  ev3::OUTPUT_D,
  cl::name("hand"),
  cl::desc("Motor port for the grabber."));

static delta_robot_args delta_geometry("", "Delta robot geometry");

static cl::arg<float> gravity(
  -1225.0f,
  cl::name("gravity"),
  cl::desc("Acceleration due to gravity, in studs/s^2."));

static cl::arg<float> intercept_delay(
  0.02f,
  cl::name("intercept-delay"),
  cl::desc("Delay between commanding the delta robot to move and moving in reality, in s."));
static cl::arg<float> catch_delay(
  0.03f,
  cl::name("catch-delay"),
  cl::desc("Delay between commanding the effector to close and closing in reality, in s."));
static cl::arg<float> observation_delay(
  0.016f,
  cl::name("observation-delay"),
  cl::desc("Delay between observing the object and reality, in s."));
static cl::arg<float> reset_delay(
  1.0f,
  cl::name("reset-delay"),
  cl::desc("Delay between initiating a catch action and returning to the ready state."));

static cl::arg<string> viz_host(
  "",
  cl::name("viz-host"),
  cl::desc("Hostname/address of the visualization server."),
  cl::group("Visualization"));
static cl::arg<short> viz_port(
  3333,
  cl::name("viz-port"),
  cl::desc("Network port of the visualization server."),
  cl::group("Visualization"));

static cl::arg<vector3f> init_x(
  vector3f(0.0f, 0.0f, 0.0f),
  cl::name("init-x"));
static cl::arg<vector3f> init_v(
  vector3f(0.0f, 0.0f, 0.0f),
  cl::name("init-v"));

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << showpos << setprecision(3);
  cerr << fixed << showpoint << showpos << setprecision(3);
    
  typedef chrono::high_resolution_clock clock;

  // Define the camera transforms.
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();
    
  // Start a thread to find the visualization server address while we start up and calibrate the robot.
  thread find_host;
  if (viz_host->empty() && viz_port != 0) {
    thread t([&]() {
      try {
        viz_host = viz_client::find_host(viz_port);
        cout << "Found visualization host at " << *viz_host << ":" << viz_port << endl;
      } catch(exception &ex) {
        dbg(1) << "viz_client::find_host failed: " << ex.what() << endl;
      }
    });
    std::swap(find_host, t);
  }
  
  observation_buffer obs0, obs1;
  float obs_t0 = 0.0f;
  mutex obs_lock;

  auto t0 = clock::now();
  thread tracking_thread([&]() {
    nxtcam nxtcam0(port_to_i2c_path(stereo.cam0.port));
    nxtcam nxtcam1(port_to_i2c_path(stereo.cam1.port)); 
    cout << "Cameras:" << endl;
    cout << nxtcam0.device_id() << " " << nxtcam0.version() << " (" << nxtcam0.vendor_id() << ")" << endl;
    cout << nxtcam1.device_id() << " " << nxtcam1.version() << " (" << nxtcam1.vendor_id() << ")" << endl;

    nxtcam0.track_objects();
    nxtcam1.track_objects();
    cout << "Tracking objects..." << endl;
  
    // t will increment in regular intervals of T.
    auto t = clock::now();
    chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
    while (true) {
      nxtcam::blob_list blobs0 = nxtcam0.blobs();
      float t_obs = chrono::duration_cast<chrono::duration<float>>(clock::now() - t0).count() + observation_delay*1e-3f;
      nxtcam::blob_list blobs1 = nxtcam1.blobs();

      obs_lock.lock();
      while(!obs0.empty() && obs_t0 + obs0.front().t + max_flight_time < t_obs) 
        obs0.pop_front();
      while(!obs1.empty() && obs_t0 + obs1.front().t + max_flight_time < t_obs)
        obs1.pop_front();
      if (obs0.empty() && obs1.empty())
        obs_t0 = t_obs;
      for (nxtcam::blob &i : blobs0) {
        obs0.push_back(observation(t_obs - obs_t0, cam0.sensor_to_focal_plane(i.center())));
        dbg(1) << "cam0 n=" << obs0.size() << ", x=" << obs0.back().f << endl;
      }
      for (nxtcam::blob &i : blobs1) {
        obs1.push_back(observation(t_obs - obs_t0, cam1.sensor_to_focal_plane(i.center())));
        dbg(1) << "cam1 n=" << obs1.size() << ", x=" << obs1.back().f << endl;
      }
      obs_lock.unlock();

      t += T;
      this_thread::sleep_until(t);
    }
  });

  // Initialize the delta robot.
  delta_hand delta(delta_geometry.geometry(), hand);
  // Set the motor parameters.
  delta.set_pid(pid->x, pid->y, pid->z);
  delta.init();

  // Bask in the glory of the calibration result for a moment.
  this_thread::sleep_for(chrono::milliseconds(500));


  // Check to see if we should connect to a visualization host.
  find_host.join();
  viz_client viz;
  if (!viz_host->empty())
    viz.connect(viz_host, viz_port);
  
  delta_robot::ellipse volume = delta.volume();

  // Use a reasonable initial guess for the trajectory.  
  trajectoryf tj_init;
  tj_init.x = vector3f(0.0f, 200.0f, 0.0f);
  tj_init.v = -tj_init.x;
  tj_init.v /= max_flight_time;
  tj_init.v.z += -0.5f*gravity*max_flight_time;

  trajectoryf tj = tj_init;
  float dt = 0.0f;
  float current_obs = 0.0f;

  // Remember the expected intercepts of the delta robot volume and the z plane. 
  struct intercept {
    float t;
    vector3f x;
  };
  const float t_none = numeric_limits<float>::infinity();

  intercept entry, exit;
  entry.t = exit.t = t_none;
  float reset_at = t_none;

  while (true) {
    float t_now = chrono::duration_cast<chrono::duration<float>>(clock::now() - t0).count();

    // Don't bother trying to estimate a new trajectory if we are closer than the 
    // observation delay to intercept, new observations will not matter if we have
    // them anyways.
    if (t_now + observation_delay < entry.t) {
      observation_buffer obs0_, obs1_;
      float obs_t0_;

      // Copy the observation buffers from the background thread.
      obs_lock.lock();
      obs_t0_ = obs_t0;
      for (size_t i = obs0.begin(); i != obs0.end(); i++)
        obs0_.push_back(obs0[i]);
      for (size_t i = obs1.begin(); i != obs1.end(); i++)
        obs1_.push_back(obs1[i]);
      obs_lock.unlock();

      if (obs0_.empty() || obs1_.empty() || obs0_.size() + obs1_.size() < 7) {
        entry.t = exit.t = t_none;
        this_thread::sleep_for(chrono::milliseconds(50));
      } else {
        bool update_tj = false;
        if (obs_t0_ + obs0_.back().t > current_obs) {
          current_obs = obs_t0 + obs0_.back().t;
          update_tj = true;
        }
        if (obs_t0_ + obs1_.back().t > current_obs) {
          current_obs = obs_t0 + obs1_.back().t;
          update_tj = true;
        }

        // Find the trajectory of the ball given the observations.
        if (update_tj) {
          try {
            entry.t = exit.t = t_none;
            estimate_trajectory(
                gravity, 
                cam0, cam1,
                obs0_, obs1_, 
                dt, tj);

            // Intersect the trajectory with the z plane, the last place on the trajectory we can reach.
            exit.t = intersect_trajectory_zplane(gravity, tj, volume.z_min);
            exit.x = tj.position(gravity, exit.t);

            // If the trajectory intercepts the z plane, find the first intercept with the volume.
            try {
              entry.t = intersect_trajectory_ellipse(gravity, tj, make_pair(volume.origin, volume.radius), 0.0f, exit.t);
              entry.x = tj.position(gravity, entry.t);
              try {
                // If the trajectory exits the ellipse before crossing the z plane, use that as the exit intercept.
                exit.t = intersect_trajectory_ellipse(gravity, tj, make_pair(volume.origin, volume.radius), entry.t + 1e-3f, exit.t);
                exit.x = tj.position(gravity, exit.t);
              } catch (runtime_error &ex) {
                // If the trajectory does not exit the ellipse before crossing the z plane, use the z plane crossing
                // as the exit intercept. It must lie in the volume to be a valid intercept.
                if (!volume.contains(exit.x))
                  throw runtime_error("z plane intercept is unreachable");                
              }

              // Adjust the intercepts from local trajectory time to global time.
              exit.t += obs_t0_;
              entry.t += obs_t0_;

              cout << "trajectory found with expected intercepts at:" << endl;
              cout << "  t=" << entry.t << " s (" << entry.t - t_now << " s) at x=" << entry.x << endl;
              cout << "  t=" << exit.t << " s (" << exit.t - t_now << " s) at x=" << exit.x << endl;
              assert(entry.t < exit.t);
            } catch(runtime_error &ex) {
              dbg(1) << ex.what() << endl;
              cout << "trajectory found with unreachable intercept expected at:" << endl;
              cout << "  t=" << exit.t << " s (" << exit.t + obs_t0 - t_now << ") s at x=" << exit.x << endl;
              entry.t = exit.t = t_none;
            }
            dbg(1) << "  trajectory x=" << tj.x << ", v=" << tj.v << endl;
          } catch(runtime_error &ex) {
            dbg(1) << ex.what() << endl;
            tj = tj_init;
            dt = 0.0f;
            delta.set_position_setpoint(volume.origin);
          }
        }
      }
    }
    try {
      if (t_now < exit.t) {
        if (t_now + intercept_delay > entry.t) {
          // If the first intercept has passed, move to the second intercept in an attempt to match the trajectory of the ball.
          if (sqr_abs(delta.position_setpoint() - exit.x) > 0.5f) {
            delta.set_position_setpoint(exit.x);
            dbg(1) << "moving to intercept entry (t=" << t_now << " s)" << endl;
            reset_at = t_now + reset_delay;
          }
        } else if (entry.t != t_none) {
          // Move to prepare for the first intercept.
          if (sqr_abs(delta.position_setpoint() - entry.x) > 0.5f) {
            delta.set_position_setpoint(entry.x);
            dbg(1) << "moving to intercept exit (t=" << t_now << " s)" << endl;
            reset_at = t_now + reset_delay;
          }
        }

        // If the current time is half way between the intercepts including the effector delay, catch the ball!
        if (t_now + catch_delay > (entry.t + exit.t)/2.0f) {
          cout << "catching the ball!" << endl;
          delta.close_hand();
          entry.t = exit.t = t_none;
          reset_at = t_now + reset_delay;

          // Clear out the trajectory data
          obs_lock.lock();
          obs0.clear();
          obs1.clear();
          obs_lock.unlock();
        }
      } else {
        throw runtime_error("intercept has passed");
      }
    } catch(runtime_error &ex) {
      // If the trajectory does not have an intercept we can reach, reset the trajectory just in case
      // it was something wacky.
      dbg(1) << ex.what() << endl;

      tj = tj_init;
      dt = 0.0f;
      entry.t = exit.t = t_none;
      delta.set_position_setpoint(volume.origin);
    }
    if (t_now > reset_at) {
      dbg(1) << "resetting..." << endl;
      delta.set_position_setpoint(volume.origin);
      this_thread::sleep_for(chrono::milliseconds(500));
      delta.open_hand();
      reset_at = t_none;
      dbg(1) << "reset done" << endl;
    }
  }

  return 0;
}
