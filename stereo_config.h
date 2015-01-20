#include "arg_port.h"
#include "camera.h"
#include "vector2.h"
#include "vector3.h"

static cl::group stereo_group("Stereo camera parameters");

struct nxtcam_config {
  arg_port port;
  cl::arg<float> orientation;
  cl::arg<vector2i> resolution;
  cl::arg<float> sensor_size;
  cl::arg<float> aspect_ratio;
  cl::arg<float> focal_length;
  cl::arg<vector2f> distortion;

  nxtcam_config(
      const std::string &prefix,
      const ev3::port_type &port,
      float orientation) : 
  port(
    port,
    cl::name(prefix + "-port"),
    cl::desc("Input port for the camera."),
    stereo_group),
  orientation(
    orientation,
    cl::name(prefix + "-orientation"),
    cl::desc("Rotation of the camera within the stereo plane, in degrees."),
    stereo_group),
  resolution(
    vector2i(176, 144),
    cl::name(prefix + "-resolution"),
    cl::desc("Resolution of the cameras, in pixels."),
    stereo_group),
  sensor_size(
    4.0f,
    cl::name(prefix + "-sensor-size"),
    cl::desc("Camera diagonal sensor size, in mm."),
    stereo_group),
  aspect_ratio(
    1.33f,
    cl::name(prefix + "-aspect-ratio"),
    cl::desc("Camera aspect ratio."),
    stereo_group),
  focal_length(
    6.5f,
    cl::name(prefix + "-focal-length"),
    cl::desc("Focal length of the camera."),
    stereo_group),
  distortion(
    vector2f(0.0f, 0.0f),
    cl::name(prefix + "-distortion"),
    cl::desc("Spherical distortion parameters of the camera."),
    stereo_group)
  {}
};

struct stereo_config {
  nxtcam_config cam0{"cam0", ev3::INPUT_1, -90.0f};
  nxtcam_config cam1{"cam1", ev3::INPUT_4, 90.0f};

  cl::arg<vector3f> origin{
      vector3f(0.0f, 13.0f, -2.0f),
      cl::name("stereo-origin"),
      cl::desc("Origin of the stereo camera coordinate system."),
      stereo_group
  };
  cl::arg<vector3f> x{
      vector3f(1.0f, 0.0f, 0.0f),
      cl::name("stereo-x"),
      cl::desc("X axis of the stereo camera coordinate system."),
      stereo_group
  };
  cl::arg<vector3f> y{
      vector3f(0.0f, cos(53.5*pi/180 + pi/2), sin(53.5*pi/180 + pi/2)),
      cl::name("stereo-y"),
      cl::desc("Y axis of the stereo camera coordinate system."),
      stereo_group
  };
  cl::arg<float> baseline{
      22.0f,
      cl::name("stereo-baseline"),
      cl::desc("Distance on the x axis between the stereo cameras.")
  };

  std::pair<cameraf, cameraf> cameras() const {
    cameraf c0, c1;
    
    c0.resolution = vector_cast<float>(*cam0.resolution);
    c0.size = cam0.sensor_size;
    c0.aspect_ratio = cam0.aspect_ratio;
    c0.focal_length = cam0.focal_length;
    c0.distortion = cam0.distortion;
    float theta0 = cam0.orientation*pi/180;
    c0.focal_plane = basis2f(vector2f(cos(theta0), sin(theta0)), 
                             vector2f(-sin(theta0), cos(theta0)));

    c1.resolution = vector_cast<float>(*cam1.resolution);
    c1.size = cam1.sensor_size;
    c1.aspect_ratio = cam1.aspect_ratio;
    c1.focal_length = cam1.focal_length;
    c1.distortion = cam1.distortion;
    float theta1 = cam1.orientation*pi/180;
    c1.focal_plane = basis2f(vector2f(cos(theta1), sin(theta1)), 
                             vector2f(-sin(theta1), cos(theta1)));

    vector3f b_2 = vector3f(baseline/2.0f, 0.0f, 0.0f);
    vector3f z = cross(*y, *x);
    z /= abs(z);

    c0.transform = basis3f(x, y, z, *origin - b_2);
    c1.transform = basis3f(x, y, z, *origin + b_2);

    return std::make_pair(c0, c1);
  }
};
