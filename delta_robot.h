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

#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H

#include <tuple>

#include <ev3cv.h>

#include <ev3/servo.h>

namespace ev3 = ev3dev;

// This is a class to control a Delta robot. Tis class assumes that the 3 primary
// joints A, B, and C are on the XY plane centered at the origin, and the arms are located
// at -30, 90, and 210 degrees. In other words, the arms are given in counter-clockwise
// order, with arm B located on the Y axis. The positive Z axis points from the
// base to the effector.
class delta_robot {
public:
  // Mechanical parameters of a delta robot.
  struct geometry {
    ev3::port_type arm0, arm1, arm2;
    float base, effector, bicep, forearm;
    int theta_max;
  };

  // Defines the working envelope of a delta robot as the intersection of
  // 3 spheres and the half space above a z axis aligned plane. This isn't
  // a perfect model of the work volume because the effector has a non-zero radius,
  // but it is a nice approximation.
  class volume {
    vector3f o0_, o1_, o2_;
    float r_;
    float z_;
    float e_;

    vector3f min_, max_;

    void init();

  public:
    volume(
        const vector3f &o0, const vector3f &o1, const vector3f &o2,
        float r, float z, float e = 1e-3f)
      : o0_(o0), o1_(o1), o2_(o2), r_(r), z_(z), e_(e) {
      init();
    }

    const vector3f &sphere(int i) const {
      switch (i) {
      case 0: return o0_;
      case 1: return o1_;
      case 2: return o2_;
      default: throw std::out_of_range("origin out of range");
      };
    }

    float radius() const { return r_; }
    float z_min() const { return z_; }
    float epsilon() const { return e_; }

    vector3f center(float z = 0.0f) const {
      vector3f c = (o0_ + o1_ + o2_)/3;
      c.z = z_*(1 - z) + (max(max(o0_.z, o1_.z), o2_.z) + r_)*z;
      return c;
    }

    std::tuple<vector3f, vector3f> bounds() const {
      return std::tie(min_, max_);
    }

    bool contains(const vector3f &x) const {
      if (x.z + e_ < z_)
        return false;
      for (int i = 0; i < 3; i++)
        if (sqr_abs(x - sphere(i)) > r_*r_ + e_)
          return false;
      return true;
    }
  };

protected:
  class arm : public servo {
  public:
    int min;

    arm(ev3::port_type p) : servo(p), min(0) {}
  };

private:
  arm arm0, arm1, arm2;

protected:
  arm *arms[3];

  // Robot geometry.
  float base;
  float effector;
  float bicep;
  float forearm;
  int theta_max;

  // Get the position of the arm, in motor position units.
  vector3i raw_position() const {
    return vector3i(
      arms[0]->position(),
      arms[1]->position(),
      arms[2]->position());
  }

  // Get the position setpoint of the arm, in motor position units.
  vector3i raw_position_sp() const {
    return vector3i(
      arms[0]->position_sp(),
      arms[1]->position_sp(),
      arms[2]->position_sp());
  }

  void set_raw_position_sp(const vector3i &x);

  bool is_raw_position_reachable(const vector3i &x) const {
    return
      0 <= x.x && x.x <= arms[0]->min &&
      0 <= x.y && x.y <= arms[1]->min &&
      0 <= x.z && x.z <= arms[2]->min;
  }

  void run() {
    for (auto i : arms)
      i->run();
  }

  // Test forward/inverse kinematics for consistency.
  void test() const;

public:
  // Constructor describes the geometry of the delta robot.
  delta_robot(const geometry &geom)
      : arm0(geom.arm0), arm1(geom.arm1), arm2(geom.arm2)
      , base(geom.base), effector(geom.effector)
      , bicep(geom.bicep), forearm(geom.forearm)
      , theta_max(geom.theta_max) {
    // Can't construct an array with an initializer list :(
    arms[0] = &arm0;
    arms[1] = &arm1;
    arms[2] = &arm2;
  }

  // Forward some useful calls to the 3 arm motors.
  void stop(bool hold = true) {
    for (auto i : arms)
      i->stop(hold);
  }
  void set_pid_K(int Kp, int Ki, int Kd) { for (auto i : arms) i->controller().set_K(Kp, Ki, Kd); }
  bool running() const {
    for (auto i : arms)
      if (i->running())
        return true;
    return false;
  }

  // Forward kinemantics: find the spatial position given raw motor positions.
  vector3f raw_to_position(const vector3i &raw) const;
  vector3f position() const { return raw_to_position(raw_position()); }
  vector3f position_sp() const { return raw_to_position(raw_position_sp()); }

  // Inverse kinematics: find the raw motor positions given a spatial location.
  vector3i position_to_raw(const vector3f &x) const;
  void set_position_sp(const vector3f &x) { set_raw_position_sp(position_to_raw(x)); }

  // Compute a conservative bounding volume of reachable effector positions. Inverse
  // kinematics is guaranteed to succeed within this volume.
  volume work_volume(float epsilon = 1e-3f) const;

  // Find the range of motion of the motors according to the encoders. Moves to the topmost position
  // first, then finds the lower limit of each arm in sequence.
  virtual void init();
};

#endif
