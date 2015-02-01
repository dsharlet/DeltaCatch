#ifndef EV3CV_MATH_PID_CONTROLLER_H
#define EV3CV_MATH_PID_CONTROLLER_H

#include <tuple>

namespace ev3cv {

// Basic PID controller:
//
// y(t) = Kp*e(t) + Ki*I(e(t), 0, t) + Kd*de(t)/dt
// e(t) = sp - x(t)
template <typename T, T period = 1>
class pid_controller {
  T sp_;
  T y_;

  T e_;
  T i_;
  
  T Kp_, Ki_, Kd_;

public:

  pid_controller(T Kp = 1, T Ki = 1, T Kd = 0) 
    : sp_(0), y_(0), e_(0), i_(0), Kp_(Kp), Ki_(Ki), Kd_(Kd) {}

  // Update the state of the PID controller. 
  // dt is the timestep, x is the current measurement of the process variable.
  const T &tick(T x, T dt) {
    if ( dt > 0) {
      T e = sp_ - x;
      // Divide out the period here, otherwise this calculation is prone to overflow.
      i_ += (e*dt)/period;
      T d = e - e_;
      e_ = e;

      y_ = Kp_*e + Ki_*i_ + (Kd_*d*period)/dt;
    } else {
      y_ = 0;
    }

    return y_;
  }
  const T &output() const { return y_; }

  // Clear the internal state of the PID controller.
  void reset() {
    sp_ = 0;
    y_ = 0;
    e_ = 0;
    i_ = 0;
  }

  // Get/set the setpoint.
  const T &setpoint() const { return sp_; }
  void set_setpoint(const T &x) { 
    sp_ = x;
  }
  
  // Get or set the PID parameters.
  std::tuple<int, int, int> K() const { return std::tie(Kp_, Ki_, Kd_); }
  void set_K(int Kp, int Ki, int Kd) { Kp_ = Kp; Ki_ = Ki; Kd_ = Kd; }
};

}  // namespace ev3cv

#endif
