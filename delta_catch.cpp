// Copyright 2015 Google, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <memory>
#include <iomanip>

#include <ev3/nxtcam.h>

#include "debug.h"
#include "delta_hand.h"
#include "trajectory.h"

#include "stereo_config.h"
#include "delta_robot_args.h"

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
  0.05f,
  cl::name("intercept-delay"),
  cl::desc("Delay between commanding the delta robot to move and moving in reality, in s."));
static cl::arg<float> catch_delay(
  0.01f,
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
static cl::arg<float> catch_z_offset(
  4.5f,
  cl::name("catch-z-offset"),
  cl::desc("Z offset from the effector position to the intercept position."));

static cl::arg<vector3f> init_x(
  vector3f(0.0f, 0.0f, 0.0f),
  cl::name("init-x"));
static cl::arg<vector3f> init_v(
  vector3f(0.0f, 0.0f, 0.0f),
  cl::name("init-v"));

float intersect_trajectory_volume(
    float gravity, const trajectoryf &tj,
    const delta_robot::volume &volume,
    float t_min, float t_max) {
  float r = volume.radius();
  for (int i = 0; i < 3; i++) {
    // Find an intersection with this sphere.
    float t = intersect_trajectory_sphere(gravity, tj, volume.sphere(i), r, t_min, t_max);

    // If all of the spheres contain the intersection, this is the intersection we care about.
    int contained = 1;
    for (int j = 0; j < 3; j++)
      if (i != j && sqr_abs(tj.position(gravity, t) - volume.sphere(j)) < sqr(r))
        contained++;
    if (contained == 3)
      return t;
  }
  throw runtime_error("no trajectory-volume intersection");
}

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << showpos << setprecision(3);
  cerr << fixed << showpoint << showpos << setprecision(3);

  typedef chrono::high_resolution_clock clock;

  // Define the camera transforms.
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();

  mutex obs_lock;
  observation_buffer obs0, obs1;
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
    chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
    while (true) {
      nxtcam::blob_list blobs0 = nxtcam0.blobs();
      auto t = clock::now();
      nxtcam::blob_list blobs1 = nxtcam1.blobs();

      float t_obs = chrono::duration_cast<chrono::duration<float>>(t - t0).count() + observation_delay*1e-3f;
      obs_lock.lock();
      while(!obs0.empty() && obs0.front().t + max_flight_time < t_obs)
        obs0.pop_front();
      while(!obs1.empty() && obs1.front().t + max_flight_time < t_obs)
        obs1.pop_front();
      for (nxtcam::blob &i : blobs0) {
        if (obs0.empty() && obs1.empty())
          dbg(1) << string(80, '-') << endl;
        obs0.push_back(observation(t_obs, cam0.sensor_to_focal_plane(i.center())));
        dbg(1) << "cam0 n=" << obs0.size() << ", t=" << t_obs << ", x=" << obs0.back().f << endl;
      }
      for (nxtcam::blob &i : blobs1) {
        if (obs0.empty() && obs1.empty())
          dbg(1) << string(80, '-') << endl;
        obs1.push_back(observation(t_obs, cam1.sensor_to_focal_plane(i.center())));
        dbg(1) << "cam1 n=" << obs1.size() << ", t=" << t_obs << ", x=" << obs1.back().f << endl;
      }
      if (obs0.empty() && obs1.empty()) {
        t0 = t;
      }
      obs_lock.unlock();

      t += T;
      this_thread::sleep_until(t);
    }
  });

  // Initialize the delta robot.
  delta_hand delta(delta_geometry.geometry(), hand);
  // Set the motor parameters.
  delta.set_pid_K(pid->x, pid->y, pid->z);
  delta.init();
  delta_robot::volume volume = delta.work_volume();
  delta.set_position_sp(volume.center(0.5f));

  // Bask in the glory of the calibration result for a moment.
  this_thread::sleep_for(chrono::milliseconds(500));

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
  auto reset_at = clock::time_point::max();

  while (true) {
    float t_now = chrono::duration_cast<chrono::duration<float>>(clock::now() - t0).count();

    // Don't bother trying to estimate a new trajectory if we are closer than the
    // observation delay to intercept, new observations will not matter if we have
    // them anyways.
    if (t_now + observation_delay < entry.t) {
      observation_buffer obs0_, obs1_;

      // Copy the observation buffers from the background thread.
      obs_lock.lock();
      auto t0_ = t0;
      for (size_t i = obs0.begin(); i != obs0.end(); i++)
        obs0_.push_back(obs0[i]);
      for (size_t i = obs1.begin(); i != obs1.end(); i++)
        obs1_.push_back(obs1[i]);
      obs_lock.unlock();

      if (obs0_.empty() || obs1_.empty()) {
        current_obs = 0.0f;
        tj = tj_init;
        dt = 0.0f;
        entry.t = exit.t = t_none;
        this_thread::sleep_for(chrono::milliseconds(50));
      } else if (obs0_.size() + obs1_.size() >= 7) {
        bool update_tj = false;
        if (obs0_.back().t > current_obs) {
          current_obs = obs0_.back().t;
          update_tj = true;
        }
        if (obs1_.back().t > current_obs) {
          current_obs = obs1_.back().t;
          update_tj = true;
        }

        // Find the trajectory of the ball given the observations.
        if (update_tj) {
          try {
            float entry_t = entry.t;
            entry.t = exit.t = t_none;
            estimate_trajectory(
                gravity,
                cam0, cam1,
                obs0_, obs1_,
                dt, tj,
                entry_t - t_now - (intercept_delay + catch_delay));

            // Update t_now because estimate_trajectory can take a while.
            t_now = chrono::duration_cast<chrono::duration<float>>(clock::now() - t0_).count();

            // Shift the trajectory down so we can treat the effector intercept position as matching the ball intercept.
            tj.x.z -= catch_z_offset;

            // Intersect the trajectory with the z plane, the last place on the trajectory we can reach.
            exit.t = intersect_trajectory_zplane(gravity, tj, volume.z_min());
            exit.x = tj.position(gravity, exit.t);

            // If the trajectory intercepts the z plane, find the first intercept with the volume.
            try {
              entry.t = intersect_trajectory_volume(gravity, tj, volume, 0.0f, exit.t);
              entry.x = tj.position(gravity, entry.t);
              try {
                // If the trajectory exits the volume before crossing the z plane, use that as the exit intercept.
                exit.t = intersect_trajectory_volume(gravity, tj, volume, entry.t + 1e-3f, exit.t);
                exit.x = tj.position(gravity, exit.t);
              } catch (runtime_error &ex) {
                // If the trajectory does not exit the ellipse before crossing the z plane, use the z plane crossing
                // as the exit intercept. It must lie in the volume to be a valid intercept.
                if (!volume.contains(exit.x))
                  throw runtime_error("z plane intercept is unreachable");
              }

              cout << "trajectory found with expected intercepts at:" << endl;
              cout << "  entry t=" << entry.t - t_now << " s at x=" << entry.x << endl;
              cout << "  exit t=" << exit.t - t_now << " s at x=" << exit.x << endl;
              assert(entry.t < exit.t);
            } catch(runtime_error &ex) {
              dbg(1) << ex.what() << endl;
              cout << "trajectory found with unreachable intercept expected at:" << endl;
              cout << "  exit t=" << exit.t + t_now << " s at x=" << exit.x << endl;
              entry.t = exit.t = t_none;
            }
            dbg(1) << "  trajectory x=" << tj.x << ", v=" << tj.v << ", dt=" << dt*sample_rate << endl;
          } catch(runtime_error &ex) {
            dbg(1) << ex.what() << endl;
            tj = tj_init;
            dt = 0.0f;
            delta.set_position_sp(volume.center(0.5f));
          }
        }
      }
    }
    try {
      if (t_now < exit.t) {
        if (t_now + intercept_delay > entry.t) {
          // If the first intercept has passed, move to the second intercept in an attempt to match the trajectory of the ball.
          if (sqr_abs(delta.position_sp() - exit.x) > 0.5f) {
            dbg(1) << "moving to intercept exit x=" << exit.x << endl;
            delta.set_position_sp(exit.x);
            reset_at = clock::now() + chrono::seconds(1);
          }
        } else if (entry.t != t_none) {
          // Move to prepare for the first intercept.
          if (sqr_abs(delta.position_sp() - entry.x) > 0.5f) {
            dbg(1) << "moving to intercept entry x=" << entry.x << endl;
            delta.set_position_sp(entry.x);
            reset_at = clock::now() + chrono::seconds(1);
          }
        }

        // If the current time is half way between the intercepts including the effector delay, catch the ball!
        if (t_now + catch_delay > (entry.t + exit.t)/2.0f) {
          cout << "catching the ball!" << endl;
          delta.close_hand();
          entry.t = exit.t = t_none;
          reset_at = clock::now() + chrono::seconds(1);

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
      delta.set_position_sp(volume.center(0.5f));
    }
    if (clock::now() > reset_at) {
      dbg(1) << "resetting..." << endl;
      reset_at = clock::time_point::max();
      tj = tj_init;
      dt = 0.0f;
      entry.t = exit.t = t_none;
      delta.set_position_sp(volume.center(0.5f));
      this_thread::sleep_for(chrono::milliseconds(500));
      delta.open_hand();
      dbg(1) << "reset done" << endl;
    }
  }

  return 0;
}
