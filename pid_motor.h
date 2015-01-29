#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include <mutex>

#include <ev3dev.h>

#include "pid_controller.h"

// Controls an EV3 motor with a PID controller.
class pid_motor {
protected:
  ev3dev::motor m_;
  // The PID controller uses units of milliseconds.
  pid_controller<int, 1000> pid_;

  std::mutex lock_;

  int max_duty_cycle_;

public:
  void tick(int ms);

public:
  pid_motor(const ev3dev::port_type &port);
  ~pid_motor();

  // Basic motor attributes.
  bool connected() const { return m_.connected(); }
  std::string port_name() const { return m_.port_name(); }
  ev3dev::device_type type() const { return m_.type(); }

  void run();
  void stop();
  void reset();

  bool running() const { return m_.running(); }
  
  ev3dev::mode_type stop_mode() const { return m_.stop_mode(); }
  void set_stop_mode(const ev3dev::mode_type &v) { m_.set_stop_mode(v); }

  // Get or set the current position of the motor.
  int position() const { return m_.position(); }
  void set_position(int p) { m_.set_position(p); }

  // Get or set the position setpoint.
  int position_setpoint() const { return pid_.setpoint(); }
  void set_position_setpoint(int sp) { pid_.set_setpoint(sp); }

  int max_duty_cycle() const { return max_duty_cycle_; }
  void set_max_duty_cycle(int x);

  // Get or set the PID parameters.
  std::tuple<int, int, int> K() const;
  void set_K(int Kp, int Ki, int Kd);
};

#endif
