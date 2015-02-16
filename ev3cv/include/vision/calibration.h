/** \file calibration.h
 * Routines for camera calibration.
 */

/** 
\page calibration Camera calibration

Camera calibration is the process of learning all of the parameters that control how light gets mapped to the sensor observations 
reported by the camera. Most standard camera calibration techniques work by presenting a pattern with features at a known relative
position that are convenient for extracting via image analysis. 

Because NXTcam and any other cameras that could reasonably be expected to connect to an EV3 do not provide images, the standard 
calibration techniques will not work. The camera calibration routines in ev3cv work by moving an object in front of a pair of stereo
cameras, where the object is constrained in some way. The easiest constraint to realize is the object must lie on a sphere. This
constraint can be implemented by attaching a string to an object detectable by the cameras, and fixing the other end of the string to
a fixed point.

\subsection procedure Procedure

ev3cv provides a utility executable called 'calibrate'. This executable has two modes: capturing calibration data, and running a
calibration to produce a stereo configuration. Here is the full process used to calibrate a pair of stereo cameras attached to an EV3:

-# Identify some point relative to the robot to be called the origin;
-# Attach a string with a length `r` of roughly one meter to an object your cameras have been trained to recognize;
  -# Fix the other end of the string to a known point relative to the origin;
  -# Run `calibrate --capture --sample-radius r --sample-center "[x y z]"`, where r is the radius of the string, and x, y, z are the 
coordinates of the other end of the string relative to the origin.

Repeat the last steps several times. 3-4 should be sufficient. After running this process several times, calibrate will have created 
a 'calibration_data' file containing measurements from the object. Note that the length 'r' above is the radius of the sphere, which
generally means to the center of the object being observed. 

When running, the calibrate application waits for measurements from the cameras. The application runs until it stops seeing the object. 
The coordinates printed on screen while the calibration is running will be highlighted when the object is approaching the edge of the 
visible area. 

Once the calibration_data file contains data from 3-4 capture sessions, you can calibrate your cameras. The first step to calibrating
your cameras is to provide an initial guess. The default initial guess contains a reasonable estimate for the intrinsic parameters for
the standard NXTcam lens. However, you will need to provide the camera position and orientation. 

The position is not calibrated, so you must provide a good estimate of the position of the cameras. Position is specified with the 
command line arguments `--cam0-position "[x0 y0 z0]"` and `--cam1-position "[x1 y1 z1]"`.

The orientation is provided via the basis vectors of the coordinate system containing the sensor. If the camera is oriented such that
the x and y coordinates of the sensor readings correspond to the X and Y axes of the global coordinate system, then the basis vectors 
would be `--cam0-x "[1 0 0]" --cam0-y "[0 1 0]"`. If the camera is rotated by 90 degrees around the z axis, then the camera basis vectors
might be `--cam0-x "[0 1 0]" --cam0-y "[-1 0 0]"`.

Once the position and initial orientation is ready, you can run `calibrate ...` with the above position and orientation arguments, 
which will produce a file 'stereo_config' containing the camera parameters resulting from the optimization.

Tips:
- It will take some practice to get good results that provide good coverage of the field of view of the cameras. I generally use the 
strategy of moving the object in a slow zig-zag pattern from one side of the field of view to the other. 
- For best results, be sure to light the object evenly from roughly the same vantage point of the cameras. This avoids biasing the 
observations due to shading due to light.

*/

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
 * to lie on the surface of a sphere with a known origin and radius. */
struct sphere_observation_set {
  vector3f center;
  float radius;
  std::vector<stereo_observation> samples;
};

/** Attempt to estimate intrinsic and extrinsic camera parameters from a set of 
 * \ref sphere_observation_set.
 *
 * The optimization is performed using Levenberg-Marquardt. 
 *
 * \param[in] sphere_observations Provides observation data.
 * \param[in] enable Defines which camera parameters are allowed to vary in the 
 * optimization. The variables of \ref camera contained in the string are optimized 
 * over.
 * \param[in, out] cam0 Parameters for camera 0. The input value is used as an initial guess.
 * \param[in, out] cam1 Parameters for camera 1. The input value is used as an initial guess.
 * @see sphere_observation_set
 */
float calibrate(
    const std::vector<sphere_observation_set> &sphere_observations,
    cameraf &cam0,
    cameraf &cam1,
    std::ostream &log = null_ostream(),
    const std::string &enable = "d1acR",
    int max_iterations = 100, float convergence_threshold = 1e-3f,
    float lambda_init = 1.0f, float lambda_decay = 0.5f);

/** Helper functions to convert to and from Rodgrigues' representation of rotations. In this
 * representation, rotation is stored in a vector \f$x\f$ representing an axis-angle (\f$\omega\f$-\f$\theta\f$)
 * rotation, where \f$\theta=|x|\f$, and \f$\omega=x/|x|\f$. 
 *
 * This representation is useful for solving optimization problems involving rotation. Note
 * that to_rodrigues does not use q.b in its result if the rotation is near the singularity of
 * axis-angle representation. This can be problematic if T is an automatic differentation type.
 */
///@{
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
///@}

/** Given a sequence of stereo observations from an unsynchronized pair of cameras, estiamte the
 * time shift as a fractional number of samples. */
float estimate_time_shift(const std::vector<stereo_observation> &obs);

/** Temporally smooth a set of observations. */
void filter_observations(std::vector<stereo_observation> &obs, float sigma);

/** Given a sequence of stereo observations from an unsynchronized pair of cameras, shift the
 * sequence of observations to be aligned. */
/// @{
void synchronize_observations(std::vector<stereo_observation> &obs, float shift);
float synchronize_observations(std::vector<stereo_observation> &obs);
/// @}

}  // namespace ev3cv

#endif