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

#ifndef EV3CV_EV3_SERVO_H
#define EV3CV_EV3_SERVO_H

#include <mutex>

#include <ev3dev.h>

#include <ev3cv.h>

namespace ev3cv {

/** Control an ev3dev::motor with a PID controller. Although ev3dev::motor uses a PID controller internally,
 * it is difficult to update the setpoint continuously without oscillation. ev3cv::servo is designed to support
 * this use case, and also provides an interface for controlling the setpoint via a callback function. */
class servo {
protected:
  ev3dev::motor m_;
  int max_duty_cycle_;

  // The PID controller uses units of milliseconds.
  pid_controller<int, 1000> pid_;
  std::function<int(int, int, int)> sp_fn_;
  int t_;

  std::mutex lock_;

protected:
  void tick(int ms);

  friend void controller_main();

public:
  servo(const ev3dev::port_type &port);
  ~servo();

  /** Basic servo attributes. */
  ///@{
  bool connected() const { return m_.connected(); }
  std::string port_name() const { return m_.port_name(); }
  ev3dev::device_type type() const { return m_.type(); }
  ///@}

  /** The state of the servo. */
  ///@{
  void run();
  bool running() const { return m_.running(); }
  ///@}

  /** Stop the servo.
   * \param[in] hold Indicate whether the motor should hold position or not */
  void stop(bool hold = true);
  /** Reset the state of the servo.
   * \param[in] position value to label the current position. */
  void reset(int position = 0);

  /** Get the current position of the servo. */
  int position() const { return m_.position(); }

  /** Get or set the position setpoint. */
  ///@{
  int position_setpoint() const;
  void set_position_setpoint(int sp);
  ///@}
  /** Set the position setpoint to a function callback f(x, t, dt).
   * Note that f will be called from another thread. */
  void set_position_setpoint(std::function<int(int, int, int)> sp_fn);

  /** Get or set the max duty cycle */
  ///@{
  int max_duty_cycle() const { return max_duty_cycle_; }
  void set_max_duty_cycle(int x);
  ///@}

  /** Get the controller being used to control this servo. */
  ///@{
  pid_controller<int, 1000> &controller() { return pid_; }
  const pid_controller<int, 1000> &controller() const { return pid_; }
  ///@}
};

}  // namespace ev3cv

#endif
