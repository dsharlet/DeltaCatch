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
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <chrono>

#include <ev3/nxtcam.h>
#include "stereo_config.h"
#include "delta_robot_args.h"

using namespace ev3dev;
using namespace std;

static cl::arg<vector3i> pid(
  vector3i(5000, 5000, 100),
  cl::name("pid"),
  cl::desc("PID parameters Kp, Ki, Kd."));

static delta_robot_args delta_geometry("", "Delta robot geometry");

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

static cl::arg<float> scale(
  1.0f,
  cl::name("scale"),
  cl::desc("Ratio of robot movement to object movement."));

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);

  nxtcam nxtcam0(port_to_i2c_path(stereo.cam0.port));
  nxtcam nxtcam1(port_to_i2c_path(stereo.cam1.port));
  cout << "Cameras:" << endl;
  cout << nxtcam0.device_id() << " " << nxtcam0.version() << " (" << nxtcam0.vendor_id() << ")" << endl;
  cout << nxtcam1.device_id() << " " << nxtcam1.version() << " (" << nxtcam1.vendor_id() << ")" << endl;

  thread nxtcam_init_thread([&] () {
    nxtcam0.track_objects();
    nxtcam1.track_objects();
    cout << "Tracking objects..." << endl;
  });

  // Initialize the delta robot.
  delta_robot delta(delta_geometry.geometry());
  if (scale > 0.0f) {
    // Set the motor parameters.
    delta.set_pid_K(pid->x, pid->y, pid->z);
    delta.init();

    // Bask in the glory of the calibration result for a moment.
    this_thread::sleep_for(chrono::milliseconds(500));
  }

  nxtcam_init_thread.join();

  delta_robot::volume volume = delta.work_volume();

  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();

  float baseline = abs(cam1.x - cam0.x);
  if (baseline < 1e-6f)
    throw runtime_error("camera baseline is zero");
  vector3f b = unit(cam1.x - cam0.x);

  // t will increment in regular intervals of T.
  typedef chrono::high_resolution_clock clock;
  auto t = clock::now();
  chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));

  vector3f origin(0.0f, 0.0f, 0.0f);

  string eraser;
  while (true) {
    nxtcam::blob_list blobs0 = nxtcam0.blobs();
    nxtcam::blob_list blobs1 = nxtcam1.blobs();

    if (blobs0.size() == 1 && blobs1.size() == 1) {
      const nxtcam::blob &b0 = blobs0.front();
      const nxtcam::blob &b1 = blobs1.front();

      vector3f x0 = cam0.sensor_to_projection(b0.center(), 1.0f) - cam0.x;
      vector3f x1 = cam1.sensor_to_projection(b1.center(), 1.0f) - cam1.x;

      // The camera focal planes may not be parallel to the baseline, so we tweak z
      // to make similar triangles with a vertex contained in the plane parallel
      // to the baseline.
      float z0 = abs(x0 - dot(x0, b)*b);
      float z1 = abs(x1 - dot(x1, b)*b);
      // Determine z from the adjusted focal plane positions via similar triangles.
      float z = baseline/(dot(x0, b)/z0 - dot(x1, b)/z1);

      // Project the points out to the distance z.
      x0 = x0*(z/z0) + cam0.x;
      x1 = x1*(z/z1) + cam1.x;
      vector3f x = (x0 + x1)/2;

      stringstream ss;
      ss << fixed << showpoint << setprecision(3);
      ss << "x=" << x << ", ||x0 - x1||=" << abs(x0 - x1) << ", z=" << z;
      string msg = ss.str();
      if (msg.length() > eraser.length())
        eraser = string(msg.length(), ' ');
      cout << msg << string(eraser.size() - msg.size(), ' ');

      if (dot(origin, origin) == 0.0f)
        origin = x*scale - volume.center();

      x = x*scale - origin;

      if (scale > 0.0f) {
        try {
          // Move to the position.
          delta.set_position_sp(x);
        } catch(runtime_error &) {

        }
      }
    } else {
      cout << eraser;
    }
    cout << "\r";
    cout.flush();

    t += T;
    this_thread::sleep_until(t);
  }

  return 0;
}
