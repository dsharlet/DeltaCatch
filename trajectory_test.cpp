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
#include <random>

#include <ev3/nxtcam.h>

#include "debug.h"
#include "trajectory.h"

#include "stereo_config.h"

using namespace std;

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));
static cl::arg<float> max_flight_time(
  1.25f,
  cl::name("max-flight-time"),
  cl::desc("The longest time allowed for a single trajectory."));
static cl::arg<float> gravity(
  -1225.0f,
  cl::name("gravity"),
  cl::desc("Acceleration due to gravity, in studs/s^2."));

static cl::group test_group("Trajectory estimation test parameters");
static cl::arg<int> test_count(
  0,
  cl::name("count"),
  cl::desc("Number of trajectories to test estimate_trajectory on."),
  test_group);
static cl::arg<float> flight_time(
  1.25f,
  cl::name("flight-time"),
  cl::desc("Flight time of test trajectories, in seconds."),
  test_group);
static cl::arg<float> target_distance(
  250.0f,
  cl::name("distance"),
  cl::desc("How far away from the target the simulated trajectories originate, in studs."),
  test_group);
static cl::arg<float> sigma_observation(
  0.0f,
  cl::name("sigma-observation"),
  cl::desc("Standard deviation of measurements at the sensor, in pixels."),
  test_group);
static cl::arg<float> false_negative_rate(
  0.1f,
  cl::name("false-negative-rate"),
  cl::desc("Probability of missed observations in simulated trajectory observations."),
  test_group);
static cl::arg<float> false_positive_rate(
  0.0f,
  cl::name("false-positive-rate"),
  cl::desc("Probability of spurious observations in simulated trajectory observations."),
  test_group);
static cl::arg<float> tolerance(
  4.0f,
  cl::name("tolerance"),
  cl::desc("Allowed error in estimated intercept for a test to be considered successful, in studs."),
  test_group);
static cl::arg<float> z_plane(
  8.0f,
  cl::name("z-plane"),
  cl::desc("z value of the plane to intersect trajectories with."));

// Test and benchmark estimate_trajectory.
int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Define the camera transforms.
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();

  // Trajectory gemoetry.
  const pair<vector3f, float> launch = { { 0.0f, target_distance, 0.0f }, 100.0f };
  const pair<vector3f, float> target = { { 0.0f, 0.0f, 12.0f }, 20.0f };

  trajectoryf tj_init;
  tj_init.x = launch.first;
  tj_init.v = target.first - launch.first;
  tj_init.v /= flight_time;
  tj_init.v.z += -0.5f*gravity*flight_time;

  dbg(1)
    << "Test trajectory max z=" << tj_init.position(gravity, flight_time/2).z
    << ", target=" << tj_init.position(gravity, intersect_trajectory_zplane(gravity, tj_init, target.first.z)) << endl;

  if (test_count > 0) {
    // Generate simulated trajectories.

    // Sampling rate of the generated observations.
    const float T = 1.0f / sample_rate;

    // Observation noise generator.
    default_random_engine rnd;
    normal_distribution<float> obs_noise(0.0f, sigma_observation);

    // Benchmarking timer duration.
    typedef chrono::high_resolution_clock clock;
    clock::duration benchmark = clock::duration::zero();
    int benchmark_count = 0;

    int fails = 0;
    float total_err = 0.0;

    cameraf cam[2] = { cam0, cam1 };
    for (int i = 0; i < test_count; i++) {
      // Use a random time shift of +/- 1 frame.
      float dt = randf(-1.0f, 1.0f)*T;

      // Generate a trajectory to test with.
      trajectoryf tj = tj_init;
      tj.x = unit(randv3f(-1.0f, 1.0f))*launch.second + launch.first;
      tj.v = unit(randv3f(-1.0f, 1.0f))*target.second + target.first - tj.x;
      tj.v /= flight_time;
      tj.v.z += -0.5f*gravity*flight_time;

      // Generate some simulated observations of the trajectory,
      // adding some random noise/false positives/false negatives.
      observation_buffer obs[2];

      for (float t = 0.0f; t <= flight_time; t += T) {
        for (int c = 0; c < 2; c++) {
          if (randf() >= false_negative_rate) {
            vector3f x = tj.position(gravity, t);
            if (cam[c].is_visible(x)) {
              vector2f ob = cam[c].project_to_sensor(x) + vector2f(obs_noise(rnd), obs_noise(rnd));
              obs[c].push_back({t, cam[c].sensor_to_focal_plane(ob)});
            }
          }
          while (randf() < false_positive_rate)
            obs[c].push_back({t, cam[c].sensor_to_focal_plane(randv2f(-1.0f, 1.0f))});
        }
      }

      try {
        // Estimate the trajectory from the random observations.
        float dt_ = 0.0f;
        trajectoryf tj_ = tj_init;
        auto start = clock::now();
        estimate_trajectory(
            gravity,
            cam[0], cam[1],
            obs[0], obs[1],
            dt_, tj_);
        auto finish = clock::now();
        benchmark_count++;

        // Check that the trajectory is within tolerance.
        vector3f intercept_tj = tj.position(gravity, intersect_trajectory_zplane(gravity, tj, target.first.z));
        vector3f intercept_tj_ = tj_.position(gravity, intersect_trajectory_zplane(gravity, tj_, target.first.z));
        vector3f err = intercept_tj - intercept_tj_;
        total_err += abs(err);
        size_t M = obs[0].size() + obs[1].size();
        if (abs(err) > tolerance || isnan(err)) {
          cerr << "Target intercept test failed, ||actual-estimated||=" << abs(err) << ", M=" << M << endl;
          cerr << "  Actual intercept=" << intercept_tj << endl;
          cerr << "  Estimated intercept=" << intercept_tj_ << endl;
          cerr << "  Actual trajectory: x=" << tj.x << ", v=" << tj.v << ", dt=" << dt << endl;
          cerr << "  Estimated trajectory: x=" << tj_.x << ", v=" << tj_.v << ", dt=" << dt_ << endl;
          fails++;
        } else {
          cout << "Target intercept success, ||actual-estimated||=" << abs(err) << ", M=" << M << endl;
          dbg(2) << "  Actual intercept=" << intercept_tj << endl;
          dbg(2) << "  Estimated intercept=" << intercept_tj_ << endl;
          dbg(1) << "  Actual trajectory: x=" << tj.x << ", v=" << tj.v << ", dt=" << dt << endl;
          dbg(1) << "  Estimated trajectory: x=" << tj_.x << ", v=" << tj_.v << ", dt=" << dt_ << endl;
          // Only include benchmark time if the optimization was successful.
          benchmark += finish - start;
        }
      } catch(exception &ex) {
        cerr << "Test fail: " << ex.what() << endl;
        fails++;
      }
    }

    if (fails > 0)
      cerr << fails << " tests failed!" << endl;

    cout
      << "estimate_trajectory accuracy=" << total_err/test_count
      << ", benchmark=" << 1e3f*static_cast<float>(benchmark.count())/benchmark_count/clock::period::den << " ms" << endl;
  } else {
    nxtcam nxtcam0(port_to_i2c_path(stereo.cam0.port));
    nxtcam nxtcam1(port_to_i2c_path(stereo.cam1.port));
    cout << "Cameras:" << endl;
    cout << nxtcam0.device_id() << " " << nxtcam0.version() << " (" << nxtcam0.vendor_id() << ")" << endl;
    cout << nxtcam1.device_id() << " " << nxtcam1.version() << " (" << nxtcam1.vendor_id() << ")" << endl;

    nxtcam0.track_objects();
    nxtcam1.track_objects();
    cout << "Tracking objects..." << endl;

    observation_buffer obs0, obs1;
    float obs_t0 = 0.0f;

    // t will increment in regular intervals of T.
    typedef chrono::high_resolution_clock clock;
    auto t = clock::now();
    auto t0 = t;
    chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
    while (true) {
      nxtcam::blob_list blobs0 = nxtcam0.blobs();
      float t_obs = chrono::duration_cast<chrono::duration<float>>(clock::now() - t0).count();
      nxtcam::blob_list blobs1 = nxtcam1.blobs();

      // If the oldest sample is near the max flight time, estimate the trajectory.
      if (!obs0.empty() && !obs1.empty() &&
          obs_t0 + max_flight_time <= t_obs) {
        cout << "Estimating trajectory... ";
        try {
          trajectoryf tj = tj_init;
          float dt = 0.0f;
          estimate_trajectory(gravity, cam0, cam1, obs0, obs1, dt, tj);
          cout << "trajectory x=" << tj.x << ", v=" << tj.v << ", dt=" << dt*sample_rate << endl;

          float t_z0 = intersect_trajectory_zplane(gravity, tj, z_plane);
          cout << "intercept z=0 at t=" << t_z0 << ", x=" << tj.position(gravity, t_z0) << endl;
        } catch (exception &ex) {
          cout << ex.what() << endl;
        }
        obs0.clear();
        obs1.clear();
      }

      while(!obs0.empty() && obs_t0 + obs0.front().t + max_flight_time < t_obs)
        obs0.pop_front();
      while(!obs1.empty() && obs_t0 + obs1.front().t + max_flight_time < t_obs)
        obs1.pop_front();
      if (obs0.empty() && obs1.empty())
        obs_t0 = t_obs;
      if (!blobs0.empty()) {
        cout << "cam0 t_obs=" << t_obs;
        for (nxtcam::blob &i : blobs0) {
          vector2f x = cam0.sensor_to_focal_plane(i.center());
          obs0.push_back(observation(t_obs - obs_t0, x));
          cout << ", x=" << x;
        }
        cout << endl;
      }
      if (!blobs1.empty()) {
        cout << "cam1 t_obs=" << t_obs;
        for (nxtcam::blob &i : blobs1) {
          vector2f x = cam1.sensor_to_focal_plane(i.center());
          obs1.push_back(observation(t_obs - obs_t0, x));
          cout << ", x=" << x;
        }
        cout << endl;
      }

      t += T;
      this_thread::sleep_until(t);
    }

    nxtcam0.stop_tracking();
    nxtcam1.stop_tracking();
  }
  return 0;
}
