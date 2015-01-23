#ifndef CAMERA_H
#define CAMERA_H

#include "vector2.h"
#include "vector3.h"
#include "quaternion.h"
#include "matrix.h"

// Mapping of 3D coordinates to the 2D projection observed by a camera.
template <typename T>
struct camera {
  // Distortion
  vector2<T> resolution;
  vector2<T> d;

  // Defines the camera calibration matrix (intrinsic parameters):
  //     [ a.x  s  t.x ]
  // K = [ 0   a.y t.y ]
  //     [ 0    0   1  ]
  vector2<T> a;
  T s;
  vector2<T> t;

  // Defines the basis of the camera space.
  quaternion<T> R;
  vector3<T> x;

  camera() : resolution(200, 100), a(1), s(0), R(1) {}
  camera(
      const vector2<T> &resolution,
      const vector2<T> &d,
      const matrix<T, 3, 3> &K,
      const quaternion<T> &R = quaternionf(1.0f, 0.0f),
      const vector3<T> &x = vector3f(0.0f)) 
    : resolution(resolution), d(d)
    , a(K(0, 0), K(1, 1)), s(K(0, 1)), t(K(0, 2), K(1, 2))
    , R(R), x(x) {
  }
  camera(
      const vector2<T> &resolution,
      const vector2<T> &d,
      const vector2<T> &sensor_size,
      const T &focal_length,
      const quaternion<T> &R = quaternionf(1.0f, 0.0f),
      const vector3<T> &x = vector3f(0.0f)) 
    : resolution(resolution), d(d)
    , a(sensor_size/(2*focal_length)), s(0), t(0, 0)
    , R(R), x(x) {
  }
      
 
  // Realize the actual K matrix.
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
    // Inverse of distortion model via newton's method.
    vector2<U> u_ = u;
    for (int i = 0; i < 2; i++) {
      vector2<U> d_uu = d*dot(u, u);
      vector2<U> fu = (u_ - u*(vector2<U>(1) - d_uu));
      vector2<U> df_du = (d_uu + U(2)*d*u - vector2<U>(1));
      u -= fu/df_du;
    }
        
    return vector2<U>(
        (u.x + U(1))*(T(0.5)*resolution.x),
        (u.y + U(1))*(T(0.5)*resolution.y));
  }

  template <typename U>
  vector2<U> sensor_to_focal_plane(const vector2<U> &px) const {
    // Normalize coordinates.
    vector2<U> u(
        px.x*(T(2)/resolution.x) - U(1),
        px.y*(T(2)/resolution.y) - U(1));
    
    // Apply distortion model.
    u *= vector2<U>(1) - d*dot(u, u);

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

  // Unproject a point on the focal plane.
  template <typename U>
  vector3<U> focal_plane_to_projection(const vector2<U> &P, const U &z) const {
    return (R*quaternion<U>(0, P.x*z, P.y*z, z)*~R).b + x;
  }
  
  template <typename U>
  vector3<U> sensor_to_projection(const vector2<U> &px, const U &z) const {
    return focal_plane_to_projection(sensor_to_focal_plane(px), z);
  }

  bool is_visible(const vector3<T> &g) const {
    if ((~R*(g - x)*R).b.z <= T(1e-6))
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
  y.d = vector_cast<T>(x.d);
  y.a = vector_cast<T>(x.a);
  y.t = vector_cast<T>(x.t);
  y.s = scalar_cast<T>(x.s);
  y.R = quaternion_cast<T>(x.R);
  y.x = vector_cast<T>(x.x);
  return y;
}

typedef camera<float> cameraf;

#endif