#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <memory>
#include <iomanip>

#include "debug.h"
#include "delta_hand.h"
#include "nxtcam.h"
#include "arg_port.h"
#include "trajectory.h"

#include "stereo_config.h"

#include "viz_client.h"

using namespace ev3dev;
using namespace std;

static cl::group motor_control("Motor control");

static cl::arg<mode_type> regulation_mode(
  "on", 
  cl::name("regulation-mode"), 
  cl::desc("One of: 'on', 'off'."),
  motor_control);
static cl::arg<int> pulses_per_second(
  700, 
  cl::name("pulses-per-second"), 
  cl::desc("Pulses/second for when --regulation-on is specified."),
  motor_control);
static cl::arg<int> duty_cycle(
  100,
  cl::name("duty-cycle"), 
  cl::desc("Duty cycle for when --regulation-on is not specified."),
  motor_control);
static cl::arg<int> ramp(
  0, 
  cl::name("ramp"), 
  cl::desc("Ramp time, in ms."),
  motor_control);

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

static cl::arg<float> max_flight_time(
  1.25f,
  cl::name("max-flight-time"),
  cl::desc("The longest time allowed for a single trajectory."));

static arg_port hand(
  ev3::OUTPUT_D,
  cl::name("hand"),
  cl::desc("Motor port for the grabber."));

// Include delta robot command line config.
#include "delta_config.h"

static cl::arg<float> gravity(
  -1225.0f,
  cl::name("gravity"),
  cl::desc("Acceleration due to gravity, in studs/s^2."));

static cl::arg<float> intercept_delay(
  20.0f,
  cl::name("intercept-delay"),
  cl::desc("Delay between commanding the delta robot to move and moving in reality, in ms."));
static cl::arg<float> catch_delay(
  30.0f,
  cl::name("catch-delay"),
  cl::desc("Delay between commanding the effector to close and closing in reality, in ms."));
static cl::arg<float> observation_delay(
  16.67f,
  cl::name("observation-delay"),
  cl::desc("Delay between observing the object and reality, in ms."));

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
      
  // Define the camera transforms.
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();
    
  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);
    
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

  // Initialize the delta robot.
  delta_hand delta(
    arm0, arm1, arm2, hand,
    base, effector, bicep, forearm, theta_max);

  // Bask in the glory of the calibration result for a moment.
  this_thread::sleep_for(chrono::milliseconds(500));

  // Set the motor parameters.
  delta.set_regulation_mode(regulation_mode);
  delta.set_pulses_per_second_setpoint(pulses_per_second);
  delta.set_duty_cycle_setpoint(duty_cycle);
  delta.set_ramp_up(ramp);
  delta.set_ramp_down(ramp);

  // Check to see if we should connect to a visualization host.
  find_host.join();
  viz_client viz;
  if (!viz_host->empty())
    viz.connect(viz_host, viz_port);
  
  volatile bool run = true;
  observation_buffer obs0, obs1;
  float obs_t0 = 0.0f;
  mutex obs_lock;

  typedef chrono::high_resolution_clock clock;
  auto t0 = clock::now();
  try {
    thread tracking_thread([&]() {
      nxtcam nxtcam0(stereo.cam0.port);
      nxtcam nxtcam1(stereo.cam1.port); 
      dbg(1) << "Cameras:" << endl;
      dbg(1) << nxtcam0.device_id() << " " << nxtcam0.version() << " (" << nxtcam0.vendor_id() << ")" << endl;
      dbg(1) << nxtcam1.device_id() << " " << nxtcam1.version() << " (" << nxtcam1.vendor_id() << ")" << endl;

      nxtcam0.track_objects();
      nxtcam1.track_objects();
  
      // t will increment in regular intervals of T.
      auto t = clock::now();
      chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
      while (run) {
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
        for (nxtcam::blob &i : blobs0)
          obs0.push_back(observation(t_obs - obs_t0, cam0.sensor_to_focal_plane(i.center())));
        for (nxtcam::blob &i : blobs1)
          obs1.push_back(observation(t_obs - obs_t0, cam1.sensor_to_focal_plane(i.center())));
        obs_lock.unlock();

        t += T;
        this_thread::sleep_until(t);
      }
    });
    
    pair<vector3f, float> volume = delta.get_volume();

    // Use a reasonable initial guess for the trajectory.
    trajectoryf tj_init;
    tj_init.v = vector3f(0.0f, 0.0f, 0.0f);
    tj_init.x = vector3f(0.0f, 80.0f, 40.0f);//volume.first - tj_init.v;
    //tj_init.v /= max_flight_time;
    tj_init.v.z += -0.5f*gravity*1.0f;//max_flight_time;

    trajectoryf tj = tj_init;
    float dt = 0.0f;
    float current_obs = 0.0f;

    // Remember the expected intercepts of the delta robot volume and the z plane. 
    struct intercept {
      float t;
      vector3f x;
    };
    intercept intercept_a, intercept_b;

    while (run) {
      if (obs0.size() + obs1.size() < 7) {
        intercept_a.t = intercept_b.t = numeric_limits<float>::infinity();
        this_thread::sleep_for(chrono::milliseconds(50));
        continue;
      }
      
      float t_now = chrono::duration_cast<chrono::duration<float>>(clock::now() - t0).count();

      // Don't bother trying to estimate a new trajectory if we are closer than the 
      // observation delay to intercept, new observations will not matter if we have
      // them anyways.
      if (t_now + observation_delay < intercept_a.t) {
        bool update_tj = false;
        if (!obs0.empty() && obs_t0 + obs0.back().t > current_obs) {
          current_obs = obs_t0 + obs0.back().t;
          update_tj = true;
        }
        if (!obs1.empty() && obs_t0 + obs1.back().t > current_obs) {
          current_obs = obs_t0 + obs1.back().t;
          update_tj = true;
        }

        // Find the trajectory of the ball given the observations.
        if (update_tj) {
          try {
            intercept_a.t = intercept_b.t = numeric_limits<float>::infinity();
            {
              lock_guard<mutex> lock(obs_lock);
              if (obs0.size() + obs1.size() > 20) {
                for (size_t i = obs0.begin(); i != obs0.end(); i++)
                  dbg(1) << "0\t" << obs0.at(i).t << "\t" << obs0.at(i).f.x << '\t' << obs0.at(i).f.y << endl;
                for (size_t i = obs1.begin(); i != obs1.end(); i++)
                  dbg(1) << "1\t" << obs1.at(i).t << "\t" << obs1.at(i).f.x << "\t" << obs1.at(i).f.y << endl;
              }
              estimate_trajectory(
                  gravity, 
                  cam0, cam1,
                  obs0, obs1, 
                  dt, tj);
            }

            // Intersect the trajectory with the z plane, the last place on the trajectory we can reach.
            intercept_b.t = intersect_trajectory_zplane(gravity, tj, volume.first.z);
            intercept_b.x = tj.position(gravity, intercept_b.t);
            intercept_b.t += obs_t0;

            // If the trajectory intercepts the z plane where we can reach it, find the first intercept with the volume.
            if (abs(intercept_b.x - volume.first) < volume.second) {
              intercept_a.t = intersect_trajectory_sphere(gravity, tj, volume, 0.0f, intercept_b.t);
              intercept_a.x = tj.position(gravity, intercept_a.t);
              intercept_a.t += obs_t0;

              dbg(2) << "trajectory found with intercept expected at t=+" << intercept_a.t - t_now << " s at x=" << intercept_a.x << endl;
            } else {
              dbg(2) << "trajectory found with unreachable intercept expected at t=+" << intercept_b.t - t_now << " s at x=" << intercept_b.x << endl;
              dbg(2) << tj.x << " " << tj.v << endl;
            }
          } catch(runtime_error &ex) {
            dbg(3) << ex.what() << endl;
            tj = tj_init;
            dt = 0.0f;
            delta.run_to(volume.first);
          }
        }
      }
      // If the intercept is later than now, don't bother trying to catch it.
      if (t_now > intercept_b.t) {
        // Move to where we expect the ball to intersect the volume.
        delta.run_to(intercept_a.x);

        // If the current time has passed the expected intercept delay at the delta volume,
        // move to the intersection with the z plane in the hopes that we roughly match the
        // trajectory of the ball.
        if (t_now + intercept_delay > intercept_a.t) {
          delta.run_to(intercept_b.x);
        }
        // If the current time is half way between the intercepts including the effector delay, catch the ball!
        if (t_now + catch_delay > (intercept_a.t + intercept_b.t)/2.0f) {
          delta.close_hand();
          intercept_a.t = intercept_b.t = numeric_limits<float>::infinity();
        }
      } else {
        // If the trajectory does not have an intercept we can reach, reset the trajectory just in case
        // it was something wacky.
        tj = tj_init;
        dt = 0.0f;
        delta.run_to(volume.first);
      } 
    }
  } catch(exception &ex) {
    cerr << ex.what() << endl;
    return -1;
  }
  
  run = false;
  return 0;
}
