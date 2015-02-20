#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>

#include "delta_robot_args.h"

using namespace ev3dev;
using namespace std;

static cl::arg<vector3i> pid(
  vector3i(5000, 5000, 100),
  cl::name("pid"),
  cl::desc("PID parameters Kp, Ki, Kd."));

// Trace out a circle
static cl::boolean circle(
  cl::name("circle"),
  cl::desc("Trace out circles."));
static cl::arg<float> speed(
  5.0f,
  cl::name("speed"),
  cl::desc("How quickly to move, in studs/s."));

static cl::boolean hold(
  cl::name("hold"),
  cl::desc("Hold position at the center."));

static cl::boolean show_path(
  cl::name("show-path"),
  cl::desc("Write coordinates of paths to stdout."));

static delta_robot_args delta_geometry("", "Delta robot geometry");

void main_show_position(delta_robot &delta) {
  delta.stop(false);
  
  const int w = 8;

  auto x0 = delta.position();
  x0 = 0;
  while (true) {
    auto x = delta.position();
    auto dx = x - x0;
    if (dot(dx, dx) > 1e-6f) {
      cout << "\rxyz=" << setw(w) << x.x << setw(w) << x.y << setw(w) << x.z
        << "   ||xy||= " << setw(w) << sqrt(x.x*x.x + x.y*x.y);
      cout.flush();
      x0 = x;
    }
    this_thread::sleep_for(chrono::milliseconds(30));
  }
}

void main_circle(delta_robot &delta) {
  static const float pi = 3.14159265f;
  const int sample_rate = 50;

  // Get the volume of the delta bot.
  delta_robot::volume volume = delta.work_volume();
  vector3f center = volume.center();
  float radius = delta_geometry.forearm - delta_geometry.base;

  float rads = 2*pi*speed/(radius*sample_rate);

  for (float theta = 0.0f; ; theta += rads) {
    // Compute a circle around the circumference of the volume.
    vector3f x(cos(theta), sin(theta), 0.0f);
    x = x*radius + center;
    if (show_path)
      cout << x << endl;
    delta.set_position_setpoint(x);
    this_thread::sleep_for(chrono::milliseconds(1000/sample_rate));
  }
}

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);
  
  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);

  delta_robot delta(delta_geometry.geometry());
  // Set the motor parameters.
  delta.set_pid_K(pid->x, pid->y, pid->z);
  delta.init();

  // Bask in the glory of the calibration result for a moment.
  this_thread::sleep_for(chrono::milliseconds(500));
  
  // Figure out what we're doing.
  if (circle) {
    main_circle(delta);
  } else if (hold) {
    this_thread::sleep_for(chrono::seconds(10));
  } else {
    main_show_position(delta);
  }

  return 0;
}