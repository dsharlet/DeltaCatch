#include <fstream>

#include <ev3dev.h>
#include <cl/arg_port.h>
#include <vision/camera.h>

static cl::group stereo_group("Stereo camera parameters");

typedef matrix<float, 2, 2> matrix2x2f;
typedef matrix<float, 3, 3> matrix3x3f;

struct camera_config {
  cl::arg_port port;
  cl::arg<vector2i> resolution;
  cl::arg<vector2f> distortion;
  cl::arg<matrix<float, 3, 3>> calibration;

  cl::arg<quaternionf> R;
  cl::arg<vector3f> x;

  camera_config(
      const std::string &prefix,
      const ev3dev::port_type &port) : 
  port(
    port,
    cl::name(prefix + "-port"),
    cl::desc("Input port for the camera."),
    stereo_group),
  resolution(
    vector2i(176, 144),
    cl::name(prefix + "-resolution"),
    cl::desc("Resolution of the cameras, in pixels."),
    stereo_group),
  distortion(
    vector2f(0.0f, 0.0f),
    cl::name(prefix + "-distortion"),
    cl::desc("Radial distortion parameters of the camera."),
    stereo_group),
  calibration(
    matrix3x3f(1.0f),
    cl::name(prefix + "-calibration"),
    cl::desc("Camera calibration matrix."),
    stereo_group),
  R(
    quaternionf(1.0f, 0.0f, 0.0f, 0.0f),
    cl::name(prefix + "-orientation"),
    cl::desc("Camera orientation quaternion."),
    stereo_group),
  x(
    vector3f(0.0f),
    cl::name(prefix + "-position"),
    cl::desc("Camera position."),
    stereo_group)
  {}

  cameraf to_camera() const {
    return cameraf(
        vector_cast<float>(*resolution),
        distortion,
        calibration,
        R,
        x);
  }
};

class stereo_config_file_arg : public cl::arg<std::string> {
public:
  stereo_config_file_arg() : cl::arg<std::string>(
      "stereo_config",
      cl::name("stereo-config")) {}

  void parse(std::list<const char *> &argv) {
    cl::arg<std::string>::parse(argv);
    
    std::ifstream file(*this);
    cl::parse(file);
  }
};

struct stereo_config {
  camera_config cam0{"cam0", ev3dev::INPUT_1};
  camera_config cam1{"cam1", ev3dev::INPUT_4};
  stereo_config_file_arg config_file;
  
  std::pair<cameraf, cameraf> cameras() const {
    return std::make_pair(cam0.to_camera(), cam1.to_camera());
  }
};
