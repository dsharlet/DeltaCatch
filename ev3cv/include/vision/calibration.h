#ifndef EV3CV_VISION_CALIBRATION_H
#define EV3CV_VISION_CALIBRATION_H

#include <vector>
#include <memory>
#include <fstream>

#include "camera.h"
#include "../ev3cv.h"

namespace ev3cv {
  
/** Stereo observations are pairs of 2D positions, usually measured from a
 * camera. */
struct stereo_observation {
  vector2f x0, x1;
};

/** Defines a series of stereo observations associated with an object constrained 
 * to lie in a sphere with a known origin and radius. */
struct sphere_observation_set {
  vector3f center;
  float radius;
  std::vector<stereo_observation> samples;
};

/** Attempt to estimate intrinsic and extrinsic camera parameters from a set of 
 * \ref sphere_observation_set. The input values of cam0 and cam1 are used as
 * an initial guess for the camera parameters. The initial guess must be roughly
 * correct for the optimization performed by this function to succeed.
 *
 * The optimization is performed using Levenberg-Marquardt. 
 *
 * @param enable allows which camera parameters are allowed to vary in the 
 * optimization. The variables of \ref camera contained in the string are optimized 
 * over.
 * @see sphere_observation_set
 */
float calibrate(
    const std::vector<sphere_observation_set> &sphere_observations,
    cameraf &cam0,
    cameraf &cam1,
    std::ostream &log = null_ostream(),
    const std::string &enable = "d1atR",
    int max_iterations = 100, float convergence_threshold = 1e-3f,
    float lambda_init = 1.0f, float lambda_decay = 0.9f);

/** Helper functions to convert to and from Rodgrigues' representation of rotations. In this
 * representation, rotation is stored in a vector x representing an axis-angle (omega-theta) 
 * rotation, where theta = ||x||, and omega = x/||x||. 
 *
 * This representation is useful for solving optimization problems involving rotation. Note
 * that to_rodrigues does not use q.b in its result if the rotation is near the singularity of
 * axis-angle representation. This can be problematic if T is an automatic differentation type.
 */
// @{
template <typename T>
vector3<T> to_rodrigues(const quaternion<T> &q) {
  // Convert to axis-angle.
  T a = T(2)*atan2(abs(q.b), q.a);
  T s = sqrt(T(1) - sqr(q.a));
  vector3<T> x;
  if (s < T(1e-6))
    x = vector3<T>(1, 0, 0);
  else
    x = q.b/s;

  return x*a;
}

template <typename T>
quaternion<T> from_rodrigues(const vector3<T> &x) {
  // abs'(x) at x = 0 is undefined, avoid that (T might be an automatic differentiation type).
  T a = sqrt(sqr_abs(x) + T(1e-6));
  return quaternion<T>(cos(a/T(2)), (x/a)*sin(a/T(2)));
}
// @}

}  // namespace ev3cv

#endif