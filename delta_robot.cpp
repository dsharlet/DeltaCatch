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
#include <algorithm>
#include <thread>

#include "debug.h"
#include "delta_robot.h"

using namespace ev3;
using namespace std;

void delta_robot::volume::init() {
  min_ = -std::numeric_limits<float>::infinity();
  max_ = std::numeric_limits<float>::infinity();
  min_.z = z_;

  for (int i = 0; i < 3; i++) {
    min_ = max(min_, sphere(i) - vector3f(r_));
    max_ = min(max_, sphere(i) + vector3f(r_));
  }
}

delta_robot::volume delta_robot::work_volume(float epsilon) const {
  static const float cos30 = sqrt(3.0f)/2;
  static const float sin30 = 0.5f;

  float b = base + cos(theta_max*pi/180)*bicep - effector;

  float z_min = -bicep + forearm;
  for (int i = 0; i < 3; i++) {
    float theta = (theta_max - arms[i]->min)*pi/180;
    z_min = max(z_min, sin(theta)*bicep + forearm);
  }

  // Position of the elbows.
  return volume(
      vector3f(-b*cos30, -b*sin30, bicep),
      vector3f( b*0.0f,   b,       bicep),
      vector3f( b*cos30, -b*sin30, bicep),
      forearm - 1,  // TODO: Why??
      z_min,
      epsilon);
}

vector3f delta_robot::raw_to_position(const vector3i &raw) const {
  vector3f theta(
    theta_max - raw.x,
    theta_max - raw.y,
    theta_max - raw.z);
  theta *= pi/180;

  static const float cos30 = sqrt(3.0f)/2;
  static const float sin30 = 0.5f;

  // Distance of the elbow to the base in the XY plane, less the effector.
  float abs_xy0 = bicep*cos(theta.x) + (base - effector);
  float abs_xy1 = bicep*cos(theta.y) + (base - effector);
  float abs_xy2 = bicep*cos(theta.z) + (base - effector);

  // Position of the elbows.
  vector3f P1(-abs_xy0*cos30, -abs_xy0*sin30, bicep*sin(theta.x));
  vector3f P2( abs_xy1*0.0f,   abs_xy1,       bicep*sin(theta.y));
  vector3f P3( abs_xy2*cos30, -abs_xy2*sin30, bicep*sin(theta.z));

  // The position of the effector is now a sphere intersection problem, there
  // are 3 spheres of radius 'forearm' at each e.
  // Construct a basis where one sphere is at 0, the next sphere is on the X axis,
  // the last sphere is on the XY plane.
  P2 -= P1;
  P3 -= P1;

  float d = abs(P2);

  vector3f X = P2 / d;
  float i = dot(X, P3);

  vector3f Y = P3 - i*X;
  Y /= abs(Y);

  float j = dot(Y, P3);

  vector3f Z = cross(Y, X);

  float x = d/2;
  float y = (i*i + j*j)/(2*j) - (i/j)*x;
  float zz = forearm*forearm - x*x - y*y;
  if (zz < 0)
    throw runtime_error("forward kinematics has no solutions");
  float z = sqrt(zz);

  return P1 + X*x + Y*y + Z*z;
}

static float position_to_raw_YZ(vector3f x0, float base, float bicep, float forearm, float effector) {
  // We want the location of the wrist relative to the shoulder.
  x0.y += effector - base;

  float A = (dot(x0, x0) + bicep*bicep - forearm*forearm)/(2*x0.z);
  float B = x0.y/x0.z;

  // Solve quadratic.
  float a = B*B + 1;
  float b = -2*A*B;
  float c = A*A - bicep*bicep;
  float D = b*b - 4*a*c;
  if (D < 0.0f)
    throw runtime_error("inverse kinematics has no solutions");

  float y = (-b + sqrt(D))/(2*a);
  float z = A - y*B;

  return atan2(z, y);
}

vector3i delta_robot::position_to_raw(const vector3f &x) const {
  static const float cos120 = -0.5f;
  static const float sin120 = sqrt(3.0f)/2;

  vector3f theta(
    position_to_raw_YZ(
      vector3f(x.x*cos120 + x.y*sin120, x.x*-sin120 + x.y*cos120, x.z),
      base, bicep, forearm, effector),
    position_to_raw_YZ(
      x,
      base, bicep, forearm, effector),
    position_to_raw_YZ(
      vector3f(x.x*cos120 + x.y*-sin120, x.x*sin120 + x.y*cos120, x.z),
      base, bicep, forearm, effector)
  );

  theta *= 180/pi;

  return vector3i(
    theta_max - static_cast<int>(floor(theta.x + 0.5f)),
    theta_max - static_cast<int>(floor(theta.y + 0.5f)),
    theta_max - static_cast<int>(floor(theta.z + 0.5f)));
}

void delta_robot::set_raw_position_sp(const vector3i &x) {
  if (!is_raw_position_reachable(x))
    throw std::runtime_error("position is unreachable");
  dbg(3) << "delta_robot setpoint -> " << x << endl;
  arms[0]->set_position_sp(x.x);
  arms[1]->set_position_sp(x.y);
  arms[2]->set_position_sp(x.z);
}

void delta_robot::init() {
  const auto timestep = chrono::milliseconds(100);
  const int stall_threshold = 50; // steps.
  const int speed = 150; // steps per second.

  dbg(1) << "initializing delta robot..." << endl;

  // Start running the motors.
  for (auto a : arms) {
    a->reset();
    a->run();

    dbg(2) << "  arm " << a->port_name() << " reset" << endl;
  }

  dbg(2) << "  finding theta_max..." << endl;
  // Set the motors to run in reverse indefinitely.
  for (auto a : arms)
    a->set_position_sp([=](int x, int t, int dt) { return (t*-speed)/1000; });

  while (is_in_transit()) {
    for (auto a : arms) {
      if (a->is_in_transit() && abs(a->position() - a->position_sp()) > stall_threshold) {
        a->reset(-1);
        a->stop();
        dbg(2) << "  found theta_max for arm " << a->port_name() << endl;
      }
    }
    this_thread::sleep_for(timestep);
  }

  // Find the lower limits of each arm.
  for (auto a : arms) {
    dbg(2) << "  finding theta_min for arm " << a->port_name() << "..." << endl;

    for (auto i : arms) {
      if (a != i) {
        int x0 = i->position();
        // Move the motor to the top.
        i->set_position_sp([=] (int x, int t, int dt) { return max(0, x0 - (t*speed)/1000); });
      } else {
        i->set_position_sp([=] (int x, int t, int dt) { return (t*speed)/1000; });
      }
      i->run();
    }

    while (true) {
      if (abs(a->position_sp() - a->position()) > stall_threshold) {
        a->min = a->position();
        dbg(2) << "  found theta_min=" << a->min << endl;
        break;
      }

      this_thread::sleep_for(timestep);
    }
  }

  // Reset the motors.
  for (auto a : arms) {
    a->run();
  }

  dbg(1) << "  done" << endl;
  set_position_sp(work_volume().center());

  // While waiting for the effector to center, run some tests.
  test();

  this_thread::sleep_for(chrono::milliseconds(50));
}

void delta_robot::test() const {
  volume v = work_volume();
  vector3f min, max;
  std::tie(min, max) = v.bounds();

  dbg(1) << "delta robot work volume min=" << min << ", max=" << max << endl;

  float tolerance = 3*abs(max - min)/100;

  int fails = 0;
  for (int i = 0; i < 100; i++) {
    vector3f x;
    do {
      x = randv3f(min, max);
    } while(!v.contains(x));

    try {
      vector3i raw = position_to_raw(x);
      if (!is_raw_position_reachable(raw)) {
        cerr << "position_to_raw gave unreachable solution at x = " << x << " (raw = " << raw << ")" << endl;
        fails++;
      }
      vector3f dx = x - raw_to_position(raw);

      if (abs(dx) > tolerance) {
        cerr << "position_to_raw not invertible at x = " << x << " (||dx|| = " << abs(dx) << ", dx = " << dx << ")" << endl;
        fails++;
      }
    } catch(runtime_error &ex) {
      cerr << "Failed to invert position x = " << x << ": " << ex.what() << endl;
      fails++;
    }
  }
  if (fails > 0)
    cerr << "Warning! " << fails << " tests failed." << endl << endl;
}
