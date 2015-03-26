// Copyright 2015 Google, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>

#include <ev3dev.h>
#include <cl/cl.h>
#include <vision/camera.h>

static cl::group stereo_group("Stereo camera parameters");

typedef matrix<float, 2, 2> matrix2x2f;
typedef matrix<float, 3, 3> matrix3x3f;

struct camera_config {
  cl::arg<std::string> port;
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
    return cameraf::from_K(
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
