#include <thread>
#include <set>
#include <signal.h>

#include <cl/cl.h>
#include <ev3/servo.h>

using namespace std;
using namespace ev3dev;

cl::arg<std::string> port(
  ev3dev::OUTPUT_A,
  cl::name("port"),
  cl::desc("Port the motor is attached to."));

cl::arg<float> period(
  4.0f,
  cl::name("period"),
  cl::desc("Sine wave period, in s."));
cl::arg<float> amplitude(
  180.0f,
  cl::name("amplitude"),
  cl::desc("Sine wave amplitude."));
cl::boolean square_wave(
  cl::name("square-wave"),
  cl::desc("Use a square wave instead of a sine wave."));

cl::boolean use_servo(
  cl::name("servo"),
  cl::desc("Use the ev3cv::servo class."));

cl::group ev3cv_group("ev3cv::servo settings");
cl::arg<vector3i> K(
  vector3i(5000, 5000, 200),
  cl::name("K"),
  cl::desc("PID parameters Kp, Ki, Kd."),
  ev3cv_group);

cl::group ev3dev_group("ev3dev::motor settings");
cl::arg<std::string> regulation_mode(
  "off",
  cl::name("regulation-mode"),
  ev3dev_group);
cl::arg<std::string> stop_mode(
  "coast",
  cl::name("stop-mode"),
  ev3dev_group);
cl::arg<int> pulses_per_second_setpoint(
  700,
  cl::name("pulses-per-second-setpoint"),
  ev3dev_group);
cl::arg<int> duty_cycle_setpoint(
  100,
  cl::name("duty-cycle-setpoint"),
  ev3dev_group);

int setpoint_fn(int ms) {
  float sp = sin(ms*2.0f*pi/(period*1000.0f));
  if (square_wave)
    sp = sp < 0.0f ? -1.0f : 1.0f;
  return sp*amplitude;
}

// Test and benchmark estimate_trajectory.
int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  typedef chrono::high_resolution_clock clock;
  chrono::milliseconds T(20);
  
  if (use_servo) {
    // Use ev3cv::servo.
    servo m(*port);
    m.controller().set_K(K->x, K->y, K->z);
    m.run();

    auto t0 = clock::now();
    for (auto t = t0; ; t += T) {
      int ms = chrono::duration_cast<chrono::milliseconds>(t - t0).count();
      m.set_position_setpoint(setpoint_fn(ms));
      this_thread::sleep_until(t);
    }
  } else {
    // To compare against the stock controller
    motor m(*port);
    m.reset();
    m.set_run_mode(motor::run_mode_position);
    m.set_regulation_mode(regulation_mode);
    m.set_pulses_per_second_setpoint(pulses_per_second_setpoint);
    m.set_duty_cycle_setpoint(duty_cycle_setpoint);
    m.set_stop_mode(stop_mode);
    
    auto t0 = clock::now();
    for (auto t = t0; ; t += T) {
      int ms = chrono::duration_cast<chrono::milliseconds>(t - t0).count();
      m.set_position_setpoint(setpoint_fn(ms));
      m.run();
      this_thread::sleep_until(t);
    }
  }
  return 0;
}

