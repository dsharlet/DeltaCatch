#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include <mutex>

#include <ev3dev.h>

#include "ev3cv/ev3cv.h"

// Controls an EV3 motor with a PID controller.
class servo {
protected:
  ev3dev::motor m_;
  int max_duty_cycle_;

  // The PID controller uses units of milliseconds.
  pid_controller<int, 1000> pid_;
  std::function<int(int, int, int)> sp_fn_;
  int t_;

  std::mutex lock_;
  
public:
  void tick(int ms);

public:
  servo(const ev3dev::port_type &port);
  ~servo();

  // Basic motor attributes.
  bool connected() const { return m_.connected(); }
  std::string port_name() const { return m_.port_name(); }
  ev3dev::device_type type() const { return m_.type(); }

  void run();
  void stop(bool hold = true);
  void reset(int position = 0);

  bool running() const { return m_.running(); }
  
  ev3dev::mode_type stop_mode() const { return m_.stop_mode(); }
  void set_stop_mode(const ev3dev::mode_type &v) { m_.set_stop_mode(v); }

  // Get or set the state of the encoder.
  int position() const { return m_.position(); }

  // Get or set the position setpoint.
  int position_setpoint() const;
  void set_position_setpoint(int sp);
  // Set the position setpoint to a function callback f(x, t, dt).
  void set_position_setpoint(std::function<int(int, int, int)> sp_fn);
  
  int max_duty_cycle() const { return max_duty_cycle_; }
  void set_max_duty_cycle(int x);

  // Get or set the PID parameters.
  std::tuple<int, int, int> K() const;
  void set_K(int Kp, int Ki, int Kd);
};

#endif
