#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H

#include "ev3dev.h"

#include "vector3.h"

namespace ev3 = ev3dev;

// Mechanical parameters of a delta robot.
struct delta_robot_geometry {
  ev3::port_type arm0, arm1, arm2;
  float base, effector, bicep, forearm;
  int theta_max;
};

// This is a class to control a Delta robot. Tis class assumes that the 3 primary 
// joints A, B, and C are on the XY plane centered at the origin, and the arms are located 
// at -30, 90, and 210 degrees. In other words, the arms are given in counter-clockwise 
// order, with arm B located on the Y axis. The positive Z axis points from the 
// base to the effector.
class delta_robot {
protected:
  class arm : public ev3::motor {
  public:
    int min;

    arm(ev3::port_type p) : ev3::motor(p), min(0) {}
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
  vector3i raw_position_setpoint() const {
    return vector3i(
      arms[0]->position_setpoint(), 
      arms[1]->position_setpoint(),
      arms[2]->position_setpoint());
  }

  void set_raw_position_setpoint(const vector3i &x) const {
    if (!is_raw_position_reachable(x))
      throw std::runtime_error("position is unreachable.");
    arms[0]->set_position_setpoint(x.x);
    arms[1]->set_position_setpoint(x.y);
    arms[2]->set_position_setpoint(x.z);
  }
  
  void run() {
    for (auto i : arms) i->run();
  }

  bool is_raw_position_reachable(const vector3i &x) const {
    return 
      0 <= x.x && x.x <= arms[0]->min &&
      0 <= x.y && x.y <= arms[1]->min &&
      0 <= x.z && x.z <= arms[2]->min;
  }
  
  // Test forward/inverse kinematics for consistency.
  void test() const;

public:
  // Constructor describes the geometry of the delta robot.
  delta_robot(const delta_robot_geometry &geom) 
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
  void set_ramp_up(int v) { for (auto i : arms) i->set_ramp_up(v); }
  void set_ramp_down(int v) { for (auto i : arms) i->set_ramp_down(v); }
  void set_duty_cycle_setpoint(int v) { for (auto i : arms) i->set_duty_cycle_setpoint(v); }
  void set_regulation_mode(const ev3::mode_type &v) { for (auto i : arms) i->set_regulation_mode(v); }
  void set_pulses_per_second_setpoint(int v) { for (auto i : arms) i->set_pulses_per_second_setpoint(v); }
  void set_stop_mode(const ev3::mode_type &v) { for (auto i : arms) i->set_stop_mode(v); }
  bool running() const {
    for (auto i : arms) 
      if (i->running()) 
        return true; 
    return false;
  }
  
  // Forward kinemantics: find the spatial position given raw motor positions.
  vector3f raw_to_position(const vector3i &raw) const;
  vector3f position() const { return raw_to_position(raw_position()); }
  vector3f position_setpoint() const { return raw_to_position(raw_position_setpoint()); }

  // Inverse kinematics: find the raw motor positions given a spatial location.
  vector3i position_to_raw(const vector3f &x) const;
  void set_position_setpoint(const vector3f &x) { set_raw_position_setpoint(position_to_raw(x)); }
  void run_to(const vector3f &x) {
    set_position_setpoint(x);
    run();
  }
  
  // Compute a conservative bounding sphere of reachable effector positions. Inverse
  // kinematics is guaranteed to succeed within the z > 0 half of this sphere.
  std::pair<vector3f, float> get_volume() const;

  // Find the range of motion of the motors according to the encoders. Moves to the topmost position 
  // first, then finds the lower limit of each arm in sequence.
  virtual void init();
};

#endif