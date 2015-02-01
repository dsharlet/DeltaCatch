#ifndef EV3CV_MATH_PID_CONTROLLER_H
#define EV3CV_MATH_PID_CONTROLLER_H

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

public:
  T Kp, Ki, Kd;

  pid_controller(T Kp = 1, T Ki = 0, T Kd = 0) 
    : sp_(0), y_(0), e_(0), i_(0), Kp(Kp), Ki(Ki), Kd(Kd) {}

  // Update the state of the PID controller. 
  // dt is the timestep, x is the current measurement of the process variable.
  const T &tick(T x, T dt) {
    if ( dt > 0) {
      T e = sp_ - x;
      // Divide out the period here, otherwise this calculation is prone to overflow.
      i_ += (e*dt)/period;
      T d = e - e_;
      e_ = e;

      y_ = Kp*e + Ki*i_ + (Kd*d*period)/dt;
    } else {
      y_ = 0;
    }

    return y_;
  }

  void reset() {
    sp_ = 0;
    y_ = 0;
    e_ = 0;
    i_ = 0;
  }

  const T &setpoint() const { return sp_; }
  void set_setpoint(const T &x) { 
    sp_ = x;
  }

  const T &output() const { return y_; }
};

#endif
