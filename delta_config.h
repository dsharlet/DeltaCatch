#include "arg_port.h"

static cl::group geometry_group("Delta robot geometry");

// Define the command line parameters to describe a delta_robot.
static arg_port arm0(
  ev3::OUTPUT_A,
  cl::name("arm0"),
  cl::desc("The motor port to drive arm 0."),
  geometry_group);
static arg_port arm1(
  ev3::OUTPUT_B,
  cl::name("arm1"),
  cl::desc("The motor port to drive arm 1."),
  geometry_group);
static arg_port arm2(
  ev3::OUTPUT_C,
  cl::name("arm2"),
  cl::desc("The motor port to drive arm 2."),
  geometry_group);
static cl::arg<float> base(
  14.5f,
  cl::name("base-radius"),
  cl::desc("Distance between the center of the base and the shoulder joints, in studs."),
  geometry_group);
static cl::arg<float> effector(
  4.5f,
  cl::name("effector-radius"),
  cl::desc("Distance between the center of the effector and the wrist joints, in studs."),
  geometry_group);
static cl::arg<float> bicep(
  15.0f,
  cl::name("bicep-length"),
  cl::desc("Distance between the shoulder and elbow joints, in studs."),
  geometry_group);
static cl::arg<float> forearm(
  28.0f,
  cl::name("forearm-length"),
  cl::desc("Distance between the elbow and wrist joints, in studs."),
  geometry_group);
static cl::arg<int> theta_max(
  100,
  cl::name("theta-max"),
  cl::desc("Theta at the shoulder joint at the reaching position, in degrees."),
  geometry_group);
