#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <thread>

#include "delta_robot.h"
#include "vector2.h"

using namespace ev3;
using namespace std;

std::pair<vector3f, float> delta_robot::get_volume() const {
  return std::make_pair(
    vector3f(0.0f, 0.0f, bicep),
    forearm - max(0.0f, base - effector));
}

vector3f delta_robot::raw_to_position(const vector3i &raw) const {
  vector3f theta(
    theta_max - raw.x,
    theta_max - raw.y,
    theta_max - raw.z);
  theta *= pi/180;

  static const float cos30 = sqrt(3.0f)/2;
  static const float sin30 = 0.5f;
  
  // Distance of the elbow to the base in the XY plane, less the effector.
  float abs_xy0 = bicep*cos(theta.x) + (base - effector);
  float abs_xy1 = bicep*cos(theta.y) + (base - effector);
  float abs_xy2 = bicep*cos(theta.z) + (base - effector);

  // Position of the elbows.
  vector3f P1(-abs_xy0*cos30, -abs_xy0*sin30, bicep*sin(theta.x));
  vector3f P2( abs_xy1*0.0f,   abs_xy1,       bicep*sin(theta.y));
  vector3f P3( abs_xy2*cos30, -abs_xy2*sin30, bicep*sin(theta.z));
    
  // The position of the effector is now a sphere intersection problem, there
  // are 3 spheres of radius 'forearm' at each e. 
  // Construct a basis where one sphere is at 0, the next sphere is on the X axis,
  // the last sphere is on the XY plane.
  P2 -= P1;
  P3 -= P1;

  float d = abs(P2);

  vector3f X = P2 / d;
  float i = dot(X, P3);

  vector3f Y = P3 - i*X;
  Y /= abs(Y);

  float j = dot(Y, P3);

  vector3f Z = cross(Y, X);

  float x = d/2;
  float y = (i*i + j*j)/(2*j) - (i/j)*x;
  float zz = forearm*forearm - x*x - y*y;
  if (zz < 0)
    throw runtime_error("forward kinematics has no solutions.");
  float z = sqrt(zz);
  
  return P1 + X*x + Y*y + Z*z;
}
   
static float position_to_raw_YZ(vector3f x0, float base, float bicep, float forearm, float effector) {
  // We want the location of the wrist relative to the shoulder.
  x0.y += effector - base;

  float A = (dot(x0, x0) + bicep*bicep - forearm*forearm)/(2*x0.z);
  float B = x0.y/x0.z;
  
  // Solve quadratic.
  float a = B*B + 1;
  float b = -2*A*B;
  float c = A*A - bicep*bicep;
  float D = b*b - 4*a*c;
  if (D < 0.0f)
    throw runtime_error("inverse kinematics has no solutions.");

  float y = (-b + sqrt(D))/(2*a);
  float z = A - y*B;
  
  return atan2(z, y);
}

vector3i delta_robot::position_to_raw(const vector3f &x) const {
  static const float cos120 = -0.5f;
  static const float sin120 = sqrt(3.0f)/2;

  vector3f theta(
    position_to_raw_YZ(
      vector3f(x.x*cos120 + x.y*sin120, x.x*-sin120 + x.y*cos120, x.z), 
      base, bicep, forearm, effector),
    position_to_raw_YZ(
      x, 
      base, bicep, forearm, effector),
    position_to_raw_YZ(
      vector3f(x.x*cos120 + x.y*-sin120, x.x*sin120 + x.y*cos120, x.z), 
      base, bicep, forearm, effector)
  );

  theta *= 180/pi;

  return vector3i(
    theta_max - static_cast<int>(floor(theta.x + 0.5f)),
    theta_max - static_cast<int>(floor(theta.y + 0.5f)),
    theta_max - static_cast<int>(floor(theta.z + 0.5f)));
}

void delta_robot::init() {
  const auto stall_time = chrono::milliseconds(200);

  // Start running the motors indefinitely, reset the position to 0.
  for (auto i : arms) {
    i->set_run_mode(motor::run_mode_forever);
    i->set_stop_mode(motor::stop_mode_hold);
    i->set_regulation_mode(motor::mode_on);
    i->set_pulses_per_second_setpoint(-200);
    i->set_position(0);
    i->run();
  }

  // Wait until all the motors hit the zero position.
  while(running()) {
    this_thread::sleep_for(stall_time);
    for (auto i : arms) {
      // If the motor is still greater than 0, assume it stopped at the limit. Otherwise, this is the new zero.
      if (i->position() >= 0)
        i->stop();
      else
        i->set_position(0);
    }
  }

  // Find the lower limits of each arm.
  for (auto i : arms) {
    for (auto j : arms) {
      if (i != j) {
        // Set the motor to hold at the top.
        j->set_run_mode(motor::run_mode_position);
        j->set_position_setpoint(0);
        j->set_stop_mode(motor::stop_mode_hold);
        j->run();
      }
    }

    i->set_pulses_per_second_setpoint(200);
    i->set_run_mode(motor::run_mode_forever);
    i->run();

    while(i->running()) {
      this_thread::sleep_for(stall_time);
      int pos = i->position();
      if (pos <= i->min) {
        i->stop();
        i->min -= 1;
        break;
      } else {
        i->min = pos;
      }
    }
  }

  // Reset the motors.
  for (auto i : arms) {
    i->set_run_mode(motor::run_mode_position);
    i->set_stop_mode(motor::stop_mode_hold);
  }

  run_to(get_volume().first);

  // While waiting for the effector to center, run some tests.
  test();

  while(running()) this_thread::sleep_for(chrono::milliseconds(50));
}

void delta_robot::test() const {
  vector3f center;
  float radius;
  tie(center, radius) = get_volume();

  int fails = 0;
  for (int i = 0; i < 100; i++) {
    vector3f x;
    do {
      x = randv3f(-1.0f, 1.0f)*radius;
    } while(abs(x) > radius);
    x.z = std::abs(x.z);
    x += center;

    try {
      vector3i raw = position_to_raw(x);
      if (!is_raw_position_reachable(raw)) {
        cerr << "position_to_raw gave unreachable solution at x = " << x << " (raw = " << raw << ")" << endl;
        fails++;
      }
      vector3f dx = x - raw_to_position(raw);

      if (abs(dx) > 3*radius/180) {
        //x.z = abs(dx);
        //cerr << x << endl;
        cerr << "position_to_raw not invertible at x = " << x << " (||dx|| = " << abs(dx) << ", dx = " << dx << ")" << endl;
        fails++;
      }
    } catch(runtime_error &ex) {
      cerr << "Failed to invert position x = " << x << ": " << ex.what() << endl;
      fails++;
    }
  }
  if (fails > 0)
    cerr << "Warning! " << fails << " tests failed." << endl << endl;
}