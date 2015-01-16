#ifndef CAMERA_H
#define CAMERA_H

#include "vector2.h"
#include "vector3.h"

// Basis of a 3D coordinate system.
template <typename T>
struct basis3 {
  vector3<T> origin, x, y, z;

  // Convert global coordinates to the local coordinates of this basis.
  // This is templatized to enable computation of Jacobians when necessary.
  template <typename U>
  vector3<U> local(vector3<U> g) const {
    g -= origin;
    return vector3<U>(dot(g, x), dot(g, y), dot(g, z));
  }

  // Convert local basis coordinates to global coorinates.
  // This is templatized to enable computation of Jacobians when necessary.
  template <typename U>
  vector3<U> global(const vector3<U> &l) const {
    return l.x*x + l.y*y + l.z*z + origin;
  }
};

// Mapping of 3D coordinates to the 2D projection observed by a camera.
template <typename T>
struct camera {
  basis3<T> basis;

  camera(
      vector3<T> origin = 0.0f,
      vector3<T> x = { 1.0f, 0.0f, 0.0f }, 
      vector3<T> y = { 0.0f, 1.0f, 0.0f }) {
    basis.origin = origin;
    basis.x = x;
    basis.y = y;
    vector3<T> z = cross(y, x);
    basis.z = z/abs(z);
  }
  
  // This is templatized to enable computation of Jacobians when necessary.
  template <typename U>
  vector2<U> project(const vector3<U> &g) const {
    // Convert the global coordinates to the local basis.
    vector3<U> l = basis.local(g);

    // Project the local coordinates.
    U inv_z = rcp(l.z);
    return vector2<U>(l.x*inv_z, l.y*inv_z);
  }
};

typedef basis3<float> basis3f;
typedef camera<float> cameraf;

#endif