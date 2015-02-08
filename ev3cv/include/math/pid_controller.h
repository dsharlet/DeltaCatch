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
  
  T deadband_;
  T i_max_;

public:
  pid_controller(T Kp = 1, T Ki = 1, T Kd = 0, T deadband = 0, T i_max = std::numeric_limits<T>::max()) 
    : sp_(0), y_(0), e_(0), i_(0), Kp_(Kp), Ki_(Ki), Kd_(Kd), deadband_(deadband), i_max_(i_max) {}

  // Update the state of the PID controller. 
  // dt is the timestep, x is the current measurement of the process variable.
  const T &tick(T x, T dt) {
    if ( dt > 0) {
      T e = sp_ - x;
      if (abs(e) >= deadband_) {
        // Divide out the period here, otherwise this calculation is prone to overflow.
        i_ += (Ki_*e*dt)/period;
        i_ = clamp(i_, -i_max_, i_max_);
        T d = e - e_;
        y_ = Kp_*e + i_ + (Kd_*d*period)/dt;
      } else {
        y_ = 0;
      }
      e_ = e;
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
  std::tuple<T, T, T> K() const { return std::tie(Kp_, Ki_, Kd_); }
  void set_K(T Kp, T Ki, T Kd) { Kp_ = Kp; Ki_ = Ki; Kd_ = Kd; }

  // Get or set the limit on the integral error state.
  T integral_limit() const { return i_max_; }
  void set_integral_limit(T x) { i_max_ = x; }

  // Get or set the deadband.
  T deadband() const { return deadband_; }
  void set_deadband(T x) { deadband_ = x; }
};

}  // namespace ev3cv

#endif
