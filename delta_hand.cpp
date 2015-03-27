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

#include <thread>

#include "delta_hand.h"

using namespace std;
using namespace ev3dev;

void delta_hand::init() {
  // Run the base configuration in a separate thread.
  thread base([=]() {
    delta_robot::init();
  });

  const auto stall_time = chrono::milliseconds(100);

  // Start running the grabber motor indefinitely.
  hand.reset();
  hand.set_run_mode(motor::run_mode_forever);
  hand.set_stop_mode(motor::stop_mode_coast);
  hand.set_regulation_mode(motor::mode_off);
  hand.set_duty_cycle_setpoint(-80);
  grab_close = hand.position();
  hand.run();

  // Wait until all the motors hit the zero position.
  while (hand.running()) {
    this_thread::sleep_for(stall_time);
    int pos = hand.position();
    if (pos >= grab_close)
      hand.stop();
    else
      grab_close = pos;
  }
  grab_open = grab_close;

  hand.set_duty_cycle_setpoint(80);
  hand.run();

  while (hand.running()) {
    this_thread::sleep_for(stall_time);
    int pos = hand.position();
    if (pos <= grab_open)
      hand.stop();
    else
      grab_open = pos;
  }

  // We always want the hand to move as quickly as possible.
  hand.set_run_mode(motor::run_mode_position);
  hand.set_duty_cycle_setpoint(100);

  // Wait for the base's init.
  base.join();
}
