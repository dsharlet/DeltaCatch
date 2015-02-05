#ifndef EV3CV_VISION_CAMERA_H
#define EV3CV_VISION_CAMERA_H

#include "../ev3cv.h"

namespace ev3cv {

/** Defines the mapping of coordinates in 3D image space through an imperfect lens
 * to a camera sensor. */
template <typename T>
struct camera {
  vector2<T> resolution;

  /** Distortion is modeled by \f$u'=u (1 + d_1 |u|^2)\f$, where \f$u'\f$ is the
   * distorted sensor observation of the normalized coordinate \$fu\$f. */
  vector2<T> d1;


  /** Defines some elements of the camera calibration matrix K 
   * 
   * TODO: Show K.
   *
   */
  // @{
  vector2<T> a;
  T s;
  vector2<T> t;
  // @}

  /** Position and orientation of the camera. */
  // @{
  quaternion<T> R;
  vector3<T> x;
  // @}

  camera() : resolution(200, 100), a(1), s(0), R(1) {}
  camera(
      const vector2<T> &resolution,
      const vector2<T> &d1,
      const vector2<T> &a,
      const T &s,
      const vector2<T> &t
      const quaternion<T> &R = quaternionf(1.0f, 0.0f),
      const vector3<T> &x = vector3f(0.0f)) 
    : resolution(resolution), d1(d1), a(a), s(s), t(t), R(R), x(x) {
  }

  /** Construct a camera from a calibration matrix. */
  static camera from_K(
      const vector2<T> &resolution,
      const vector2<T> &d1,
      const matrix<T, 3, 3> &K,
      const quaternion<T> &R = quaternionf(1.0f, 0.0f),
      const vector3<T> &x = vector3f(0.0f)) {
    return camera(
        resolution,
        d1,
        vector2<T>(K(0, 0), K(1, 1)),
        K(0, 1),
        vector2<T>(K(0, 2), K(1, 2)),
        R,
        x);
  }

  /** Construct a camera from lens and sensor information. */
  static camera from_lens(
      const vector2<T> &resolution,
      const vector2<T> &d1,
      const vector2<T> &sensor_size,
      const T &focal_length,
      const quaternion<T> &R = quaternionf(1.0f, 0.0f),
      const vector3<T> &x = vector3f(0.0f)) {
    return camera(
        resolution,
        d1,
        sensor_size/(2*focal_length),
        0,
        vector2<T>(0, 0),
        R,
        x);
  }
 
  // Realize the actual K matrix from the intrinsic parameters.
  matrix<T, 3, 3> K() const {
    matrix<T, 3, 3> k;
    k(0, 0) = a.x; k(0, 1) = s;   k(0, 2) = t.x;
                   k(1, 1) = a.y; k(1, 2) = t.y;
                                  k(2, 2) = 1;
    return k;
  }

  template <typename U>
  vector2<U> focal_plane_to_sensor(const vector2<U> &P) const {
    // Apply camera calibration matrix.
    vector2<U> u(
        a.x*P.x + s*P.y + t.x,
        a.y*P.y + t.y);
    
    // Apply distortion correction.
    u *= vector2<T>(1) + d1*dot(u, u);

    return vector2<U>(
        (u.x + T(1))*(T(0.5)*resolution.x),
        (u.y + T(1))*(T(0.5)*resolution.y));
  }

  template <typename U>
  vector2<U> sensor_to_focal_plane(const vector2<U> &px) const {
    // Normalize coordinates.
    vector2<U> u(
        px.x*(T(2)/resolution.x) - T(1),
        px.y*(T(2)/resolution.y) - T(1));
    
    // Apply distortion model.
    // Compute inverse of distortion model via newton's method.
    // TODO: Try to optimize this... lots of FLOPs here if U is a diff<>.
    vector2<U> u_ = u;
    for (int i = 0; i < 3; i++) {
      vector2<U> d = vector2<T>(1) + d1*dot(u, u);
      vector2<U> fu = u*d - u_;
      vector2<U> df_du = d + T(2)*d1*u*u;
      u -= fu/df_du;
    }

    // Solve K*x = u.
    U y = (u.y - t.y)*rcp(a.y);
    U x = (u.x - t.x - s*y)*rcp(a.x);
    return vector2<U>(x, y);
  }

  // Project to normalized coordinates to the focal plane.
  template <typename U>
  vector2<U> project_to_focal_plane(const vector3<U> &g) const {
    // Convert the global coordinates to the local transform.
    vector3<U> l = (~quaternion_cast<U>(R)*(g - vector_cast<U>(x))*quaternion_cast<U>(R)).b;

    // Project the local coordinates.
    return vector2<U>(l.x, l.y)*rcp(l.z);
  }

  template <typename U>
  vector2<U> project_to_sensor(const vector3<U> &g) const {
    return focal_plane_to_sensor(project_to_focal_plane(g));
  }

  // Unproject a point on the plane containing z.
  template <typename U>
  vector3<U> focal_plane_to_projection(const vector2<U> &P, const U &z) const {
    return (R*quaternion<U>(0, P.x*z, P.y*z, z)*~R).b + x;
  }
  
  template <typename U>
  vector3<U> sensor_to_projection(const vector2<U> &px, const U &z) const {
    return focal_plane_to_projection(sensor_to_focal_plane(px), z);
  }

  // Test if a point is visible to this camera.
  bool is_visible(const vector3<T> &g) const {
    // Unfortunately, positive z is behind the camera, not in front.
    if ((~R*(g - x)*R).b.z >= T(-1e-6))
      return false;
    vector2<T> px = project_to_sensor(g);
    return T(0) <= px.x && px.x < resolution.x && 
           T(0) <= px.y && px.y < resolution.y;
  }
};

template <typename T, typename U>
camera<T> camera_cast(const camera<U> &x) {
  camera<T> y;
  y.resolution = vector_cast<T>(x.resolution);
  y.d1 = vector_cast<T>(x.d1);
  y.a = vector_cast<T>(x.a);
  y.t = vector_cast<T>(x.t);
  y.s = scalar_cast<T>(x.s);
  y.R = quaternion_cast<T>(x.R);
  y.x = vector_cast<T>(x.x);
  return y;
}

typedef camera<float> cameraf;

}  // namespace ev3cv

#endif