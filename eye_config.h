#include "arg_port.h"
#include "camera.h"

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
static cl::arg<float> eye_focal_length(
  3.6f,
  cl::name("eye-focal-length"),
  cl::desc("Camera focal length, in mm."));
static cl::arg<vector2f> eye_sensor_size(
  vector2f(3.1f, 2.5f),
  cl::name("eye-sensor-size"),
  cl::desc("Camera sensor size, in mm."));
static cl::arg<vector2i> eye_resolution(
  vector2i(188, 144),
  cl::name("eye-resolution"),
  cl::desc("Resolution of the cameras, in pixels."));
static cl::arg<float> eye_sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

