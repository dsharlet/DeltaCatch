#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>

#include "delta_robot.h"
#include "arg_port.h"

using namespace ev3dev;
using namespace std;

static cl::boolean regulation_on(
  cl::name("regulation-on"), 
  cl::desc("Set regulation mode to on."));
static cl::arg<int> pulses_per_second(
  700, 
  cl::name("pulses-per-second"), 
  cl::desc("Pulses/second for when --regulation-on is specified."));
static cl::arg<int> duty_cycle(
  100,
  cl::name("duty-cycle"), 
  cl::desc("Duty cycle for when --regulation-on is not specified."));
static cl::arg<int> ramp(
  0, 
  cl::name("ramp"), 
  cl::desc("Ramp time, in ms."));

// Trace out a n-sided polygon horizontally.
static cl::arg<int> ngon_n(
  0,
  cl::name("ngon-n"),
  cl::desc("Repeatedly trace out an n-sided polygon with the effector."));
static cl::arg<float> ngon_r(
  0.0f,
  cl::name("ngon-r"),
  cl::desc("Radius of the n-gon to trace out, in studs."));
static cl::arg<float> ngon_z(
  0.0f,
  cl::name("ngon-z"),
  cl::desc("z axis offset from volume center of the n-gon to trace out, in studs."));

// Trace out random lines broken into steps.
static cl::arg<float> lines_dx(
  0.0f,
  cl::name("lines-dx"),
  cl::desc("Generate and move along random lines, split into steps of 'lines-dx' studs."));

// Generic options.
static cl::arg<int> pause(
  0,
  cl::name("pause"),
  cl::desc("How long to pause in between each test position, in ms."));
static cl::boolean show_path(
  cl::name("show-path"),
  cl::desc("Write coordinates of paths to stdout."));

// Include delta robot command line config.
#include "delta_config.h"

void main_show_position(delta_robot &delta) {
  delta.set_stop_mode(motor::stop_mode_coast);
  
  const int w = 8;

  auto x0 = delta.position();
  x0 = 0;
  while (true) {
    auto x = delta.position();
    auto dx = x - x0;
    if (dot(dx, dx) > 1e-6f) {
      cout << "xyz=" << setw(w) << x.x << setw(w) << x.y << setw(w) << x.z
        << "   ||xy||= " << setw(w) << sqrt(x.x*x.x + x.y*x.y)
        << endl;
      x0 = x;
    }
    this_thread::sleep_for(chrono::milliseconds(pause));
  }
}

void main_ngon(delta_robot &delta) {
  static const float pi = 3.14159265f;

  // Get the volume of the delta bot.
  vector3f center;
  float radius;
  tie(center, radius) = delta.get_volume();
  
  center.z += ngon_z;

  if (ngon_r > 0.0f)
    radius = ngon_r;

  delta.set_stop_mode(motor::stop_mode_hold);

  for (int i = 0; ; i++) {
    float theta = 2*pi*i/ngon_n + pi/2;
    
    // Compute a circle around the circumference of the volume.
    vector3f x(cos(theta), sin(theta), 0.0f);
    x = x*radius + center;

    if (show_path)
      cout << x << endl;

    delta.run_to(x);
    while (delta.running())
      this_thread::sleep_for(chrono::milliseconds(5));

    if (pause > 0)
      this_thread::sleep_for(chrono::milliseconds(pause));
  }
}

static float randf() {
  return (float)rand() / (float)RAND_MAX;
}

void main_random_lines(delta_robot &delta) {
  // Get the volume of the delta bot.
  vector3f center;
  float radius;
  tie(center, radius) = delta.get_volume();
  
  delta.set_stop_mode(motor::stop_mode_hold);

  while(true) {
    if (pause > 0)
      this_thread::sleep_for(chrono::milliseconds(pause));

    vector3f x0 = delta.position();

    // Pick a random point.
    vector3f x1 = x0;
    for (int i = 0; i < 20 && abs(x1 - x0) < radius*1.5f; i++) {
      x1 = vector3f(2*randf() - 1, 2*randf() - 1, randf());
      x1 = radius*x1/abs(x1) + center;
    };
    if (show_path)
      cout << x1 << endl;

    float steps = ceil(abs(x1 - x0)/lines_dx);
    vector3f dx = (x1 - x0)/steps;

    for (float i = 0; i < steps; i += 1.0f) {
      delta.run_to(x0 + dx*i);
      while (delta.running())
        this_thread::sleep_for(chrono::milliseconds(5));
    }
  }
}

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);
  
  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);

  delta_robot delta(
    arm0, arm1, arm2, 
    base, effector, bicep, forearm, theta_max);

  // Bask in the glory of the calibration result for a moment.
  this_thread::sleep_for(chrono::milliseconds(500));

  // Set the motor parameters.
  delta.set_regulation_mode(regulation_on ? motor::mode_on : motor::mode_off);
  delta.set_pulses_per_second_setpoint(pulses_per_second);
  delta.set_duty_cycle_setpoint(duty_cycle);
  delta.set_ramp_up(ramp);
  delta.set_ramp_down(ramp);

  // Figure out what we're doing.
  if (ngon_n > 2) {
    main_ngon(delta);
  } else if (lines_dx > 0.0f) {
    main_random_lines(delta);
  } else {
    main_show_position(delta);
  }

  return 0;
}