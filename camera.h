#ifndef CAMERA_H
#define CAMERA_H

#include "basis.h"

// Mapping of 3D coordinates to the 2D projection observed by a camera.
template <typename T>
struct camera {
  vector2<T> resolution;
  T size;
  T aspect_ratio;
  T focal_length;
  basis2<T> focal_plane;
  vector2<T> distortion;
  basis3<T> transform;

  vector2<T> sensor_size() const {
    T s = size/sqrt(sqr(aspect_ratio) + T(1));
    return vector2<T>(size*s, s);
  }
  
  template <typename U>
  vector2<U> sensor_to_normalized(vector2<U> x) const {
    // Normalize coordinates.
    x.x = x.x*(T(2)/resolution.x) - U(1);
    x.y = x.y*(T(-2)/resolution.y) + U(1);

    // Apply distortion correction.
    // Inverse of distortion via newton's method.
    vector2<U> y = x;
    for (int i = 0; i < 3; i++) {
      vector2<U> a_x2 = distortion*dot(x, x);
      vector2<U> fx = (y - x*(vector2<U>(1) - a_x2));
      vector2<U> df_dx = (a_x2 + 2*distortion*x - vector2<U>(1));
      x -= fx/df_dx;
    }

    return x;
  }
  
  template <typename U>
  vector2<U> normalized_to_sensor(vector2<U> x) const {
    // Apply distortion.
    x *= vector2<U>(1) - distortion*dot(x, x);
        
    x.x = (x.x + T(1))*(T(0.5)*resolution.x);
    x.y = (T(1) - x.y)*(T(0.5)*resolution.y);

    return x;
  }

  template <typename U>
  vector2<U> normalized_to_focal_plane(vector2<U> x) const {
    x = focal_plane.to_global(x);
    x *= sensor_size()/(T(2)*focal_length);   

    return x;
  }

  template <typename U>
  vector2<U> sensor_to_focal_plane(const vector2<U> &x) const {
    return normalized_to_focal_plane(sensor_to_normalized(x));
  }
  
  template <typename U>
  vector2<U> focal_plane_to_normalized(vector2<U> x) const {
    x /= sensor_size()/(T(2)*focal_length);  
    x = focal_plane.to_local(x);

    return x;
  }
  

  template <typename U>
  vector2<U> focal_plane_to_sensor(const vector2<U> &x) const {
    return normalized_to_sensor(focal_plane_to_normalized(x));
  }

  // Project to normalized coordinates to the focal plane.
  template <typename U>
  vector2<U> project_to_focal_plane(const vector3<U> &g) const {
    // Convert the global coordinates to the local transform.
    vector3<U> l = transform.to_local(g);

    // Project the local coordinates.
    return vector2<U>(l.x, l.y)*rcp(l.z);
  }
  
  template <typename U>
  vector2<U> project_to_normalized(const vector3<U> &g) const {
    return focal_plane_to_normalized(project_to_focal_plane(g));
  }

  template <typename U>
  vector2<U> project_to_sensor(const vector3<U> &g) const {
    return focal_plane_to_sensor(project_to_focal_plane(g));
  }
  // Unproject a point on the focal plane.
  template <typename U>
  vector3<U> focal_plane_to_projection(const vector2<U> &x, const U &z) const {
    return transform.to_global(vector3<U>(x.x*z, x.y*z, z));
  }
  
  template <typename U>
  vector3<U> normalized_to_projection(const vector2<U> &x, const U &z) const {
    return focal_plane_to_projection(normalized_to_focal_plane(x), z);
  }
  template <typename U>
  vector3<U> sensor_to_projection(const vector2<U> &x, const U &z) const {
    return focal_plane_to_projection(sensor_to_focal_plane(x), z);
  }
};

template <typename T, typename U>
camera<T> camera_cast(const camera<U> &x) {
  camera<T> y;
  y.resolution = vector_cast<T>(x.resolution);
  y.size = scalar_cast<T>(x.size);
  y.aspect_ratio = scalar_cast<T>(x.aspect_ratio);
  y.focal_length = scalar_cast<T>(x.focal_length);
  y.focal_plane = basis_cast<T>(x.focal_plane);
  y.distortion = vector_cast<T>(x.distortion);
  y.transform = basis_cast<T>(x.transform);
  return y;
}

typedef camera<float> cameraf;

#endif