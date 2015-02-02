#ifndef EV3CV_VISION_CALIBRATION_H
#define EV3CV_VISION_CALIBRATION_H

#include <vector>
#include <memory>
#include <fstream>

#include "camera.h"
#include "../ev3cv.h"

namespace ev3cv {

static std::ostream &null_ostream() {
  static std::ofstream os;
  return os;
}

// A corresponding 2D observations of a 3D point.
struct stereo_observation {
  vector2f x0, x1;
};

// A set of stereo observations of 3D points known to lie in a (possibly unknown) sphere.
struct sphere_observation_set {
  vector3f center;
  float radius;
  std::vector<stereo_observation> samples;
};

// Attempt to estimate the camera parameters from a set of sphere observations. 
float calibrate(
    std::vector<sphere_observation_set> &sphere_observations,
    cameraf &cam0,
    cameraf &cam1,
    std::ostream &log = null_ostream(),
    const std::string &enable = "d1astR",
    float lambda_init = 1.0f, float lambda_decay = 0.9f,
    float convergence_threshold = 1e-3f, int max_iterations = 100);

}  // namespace ev3cv

#endif