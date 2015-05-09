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

  void close_hand() { 
    hand.set_position_sp(grab_close); 
    hand.set_command(ev3dev::motor::command_run_to_abs_pos); 
  }

  void open_hand() { 
    hand.set_position_sp(grab_open); 
    hand.set_command(ev3dev::motor::command_run_to_abs_pos); 
  }
};

#endif
