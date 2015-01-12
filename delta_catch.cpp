#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>

#include "debug.h"
#include "delta_hand.h"
#include "nxtcam.h"
#include "arg_port.h"
#include "trajectory.h"
#include "camera.h"

using namespace ev3dev;
using namespace std;

static const float pi = 3.1415926535897f;

static cl::arg<mode_type> regulation_mode(
  "on", 
  cl::name("regulation-mode"), 
  cl::desc("One of: 'on', 'off'."));
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

// Camera geometry.
static arg_port eye0(
  ev3::INPUT_1,
  cl::name("eye0"),
  cl::desc("Input port for the first camera."));
static arg_port eye1(
  ev3::INPUT_4,
  cl::name("eye1"),
  cl::desc("Input port for the second camera."));
static cl::arg<float> eye_baseline(
  20.0f,
  cl::name("eye-baseline"),
  cl::desc("Distance between the cameras."));
static cl::arg<float> eye_y(
  13.0f,
  cl::name("eye-y"),
  cl::desc("Camera baseline y coordinate."));
static cl::arg<float> eye_z(
  -2.0f,
  cl::name("eye-z"),
  cl::desc("Camera baseline z coordinate."));
static cl::arg<float> eye_pitch(
  53.5f,
  cl::name("eye-pitch"),
  cl::desc("Camera pitch from horizontal."));
static cl::arg<float> eye_fov(
  43.5f,
  cl::name("eye-fov"),
  cl::desc("Horizontal camera field of view, in degrees."));
static cl::arg<float> eye_aspect_ratio(
  1.0f,
  cl::name("eye-aspect-ratio"),
  cl::desc("Camera aspect ratio."));

static arg_port hand(
  ev3::OUTPUT_D,
  cl::name("hand"),
  cl::desc("Motor port for the grabber."));

// Include delta robot command line config.
#include "delta_config.h"

static cl::arg<float> g(
  -1225.0f,
  cl::name("gravity"),
  cl::desc("Acceleration due to gravity, in studs/s^2."));
static cl::arg<float> sigma_observation(
  0.01f,
  cl::name("sigma-observation"),
  cl::desc("Standard deviation of observation noise in normalized projected camera space."));
static cl::arg<float> outlier_threshold(
  6.0f,
  cl::name("outlier-threshold"),
  cl::desc("Tolerance of standard error before an observation is considered an outlier."));

static cl::arg<string> viz_server(
  "",
  cl::name("viz-host"),
  cl::desc("Hostname/address of the visualization server."));
static cl::arg<int> viz_port(
  3333,
  cl::name("viz-port"),
  cl::desc("Network port of the visualization server."));


int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);
  
  // Reduce clutter of insignificant digits.
  //cout << fixed << showpoint << setprecision(3);
  //cerr << fixed << showpoint << setprecision(3);
  
  // Define the camera transforms.
  vector3f X(-1.0f, 0.0f, 0.0f);
  vector3f Y(0.0f, cos(eye_pitch*pi/180 + pi/2), sin(eye_pitch*pi/180 + pi/2));
  camera Tcam0 = {{eye_baseline/2.0f, eye_y, eye_z}, X, Y};
  camera Tcam1 = {{-eye_baseline/2.0f, eye_y, eye_z}, X, Y};

  test_estimate_trajectory(g, sigma_observation, outlier_threshold, Tcam0, Tcam1);
  
  nxtcam cam0(eye0);
  nxtcam cam1(eye1);

  dbg(1) << "Cameras:" << endl;
  dbg(1) << cam0.device_id() << " " << cam0.version() << " (" << cam0.vendor_id() << ")" << endl;
  dbg(1) << cam1.device_id() << " " << cam1.version() << " (" << cam1.vendor_id() << ")" << endl;
  
  // Initialize the delta robot.
  delta_hand delta(
    arm0, arm1, arm2, hand,
    base, effector, bicep, forearm, theta_max);

  // Bask in the glory of the calibration result for a moment.
  this_thread::sleep_for(chrono::milliseconds(500));

  // Set the motor parameters.
  delta.set_regulation_mode(regulation_mode);
  delta.set_pulses_per_second_setpoint(pulses_per_second);
  delta.set_duty_cycle_setpoint(duty_cycle);
  delta.set_ramp_up(ramp);
  delta.set_ramp_down(ramp);

  return 0;
}
