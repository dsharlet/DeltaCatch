#include <thread>
#include <set>
#include <signal.h>

#include <cl/cl.h>
#include <ev3/servo.h>

using namespace std;
using namespace ev3dev;

cl::arg<std::string> output_port(
  ev3dev::OUTPUT_A,
  cl::name("output-port"),
  cl::desc("Port the motor is attached to."));

cl::arg<std::string> input_port(
  ev3dev::OUTPUT_D,
  cl::name("input-port"),
  cl::desc("Port the tacho is attached to."));

cl::boolean use_servo(
  cl::name("servo"),
  cl::desc("Use the ev3cv::servo class."));

cl::arg<float> scale(
  1.0f,
  cl::name("scale"),
  cl::desc("Relative scale of the motion between the input and output."));

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
  "hold",
  cl::name("stop-mode"),
  ev3dev_group);
cl::arg<int> ramp_up(
  0,
  cl::name("ramp-up"),
  ev3dev_group);
cl::arg<int> ramp_down(
  0,
  cl::name("ramp-down"),
  ev3dev_group);
cl::arg<int> pulses_per_second_setpoint(
  700,
  cl::name("pulses-per-second-setpoint"),
  ev3dev_group);
cl::arg<int> duty_cycle_setpoint(
  100,
  cl::name("duty-cycle-setpoint"),
  ev3dev_group);

// Test and benchmark estimate_trajectory.
int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  typedef chrono::high_resolution_clock clock;
  chrono::milliseconds T(20);
  
  motor in(*input_port);
  in.reset();

  cout << "Turn the motor connected to port " << *input_port << "..." << endl;

  if (use_servo) {
    // Use ev3cv::servo.
    servo m(*output_port);
    m.controller().set_K(K->x, K->y, K->z);
    m.run();

    for (auto t = clock::now(); ; t += T) {
      m.set_position_setpoint(in.position()*scale);
      this_thread::sleep_until(t);
    }
  } else {
    // Compare against the stock controller
    motor m(*output_port);
    m.reset();
    m.set_run_mode(motor::run_mode_position);
    m.set_regulation_mode(regulation_mode);
    m.set_pulses_per_second_setpoint(pulses_per_second_setpoint);
    m.set_duty_cycle_setpoint(duty_cycle_setpoint);
    m.set_stop_mode(stop_mode);
    m.set_ramp_up(ramp_up);
    m.set_ramp_down(ramp_down);
    
    for (auto t = clock::now(); ; t += T) {
      m.set_position_setpoint(in.position()*scale);
      m.run();
      this_thread::sleep_until(t);
    }
  }
  return 0;
}

