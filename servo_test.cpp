#include <thread>
#include <set>
#include <signal.h>

#include "arg_port.h"
#include "servo.h"

using namespace std;

arg_port port(
  ev3::OUTPUT_A,
  cl::name("port"),
  cl::desc("Port the motor is attached to."));

cl::arg<vector3i> K(
  vector3i(1000, 10, 0),
  cl::name("K"),
  cl::desc("PID parameters Kp, Ki, Kd."));

cl::arg<float> period(
  3.0f,
  cl::name("period"),
  cl::desc("Sine wave period, in s."));
cl::arg<float> amplitude(
  700.0f,
  cl::name("amplitude"),
  cl::desc("Sine wave amplitude."));

// Test and benchmark estimate_trajectory.
int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  servo m(*port);
  m.set_K(K->x, K->y, K->z);
  m.run();

  typedef chrono::high_resolution_clock clock;
  chrono::milliseconds T(20);
  auto t0 = clock::now();
  for (auto t = t0; ; t += T) {
    int ms = chrono::duration_cast<chrono::milliseconds>(t - t0).count();
    int sp = (int)(sin(ms*2.0f*pi/(period*1000.0f))*amplitude);
    m.set_position_setpoint(sp);
    this_thread::sleep_until(t);
  }
  return 0;
}