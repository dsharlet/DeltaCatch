#ifndef DELTA_HAND_H
#define DELTA_HAND_H

#include "delta_robot.h"

// A delta robot, with an additional motor for controlling a grabber hand.
class delta_hand : public delta_robot {
protected:
  ev3::motor hand;
  int grab_open, grab_close;

public:
  delta_hand(const delta_robot::geometry &geom, ev3::port_type hand) 
    : delta_robot(geom), hand(hand), grab_open(0), grab_close(0) {} 
  
  void init();
  
  void close_hand() { hand.set_position_setpoint(grab_close); hand.run(); }
  void open_hand() { hand.set_position_setpoint(grab_open); hand.run(); }
};

#endif