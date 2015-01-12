#ifndef DELTA_HAND_H
#define DELTA_HAND_H

#include "delta_robot.h"

// A delta robot, with an additional motor for controlling a grabber hand.
class delta_hand : public delta_robot {
protected:
  ev3::motor hand;
  int grab_open, grab_close;

public:
  delta_hand(
      ev3::port_type a, ev3::port_type b, ev3::port_type c, ev3::port_type hand,
      float base, float effector, float bicep, float forearm, int theta_max,
      bool find_limits_now = true);

  void find_limits();
  
  void close_hand() { hand.set_position_setpoint(grab_close); hand.run(); }
  void open_hand() { hand.set_position_setpoint(grab_open); hand.run(); }
};

#endif