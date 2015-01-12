#ifndef CAMERA_H
#define CAMERA_H

#include "vector2.h"
#include "vector3.h"

// Basis of a 3D coordinate system.
struct basis3f {
  vector3f origin, x, y, z;

  // Convert global coordinates to the local coordinates of this basis.
  // This is templatized to enable computation of Jacobians when necessary.
  template <typename T>
  vector3<T> local(vector3<T> g) const {
    g -= origin;
    return vector3<T>(dot(g, x), dot(g, y), dot(g, z));
  }

  // Convert local basis coordinates to global coorinates.
  // This is templatized to enable computation of Jacobians when necessary.
  template <typename T>
  vector3<T> global(const vector3<T> &l) const {
    return l.x*x + l.y*y + l.z*z + origin;
  }
};

// Mapping of 3D coordinates to the 2D projection observed by a camera.
struct camera {
  basis3f basis;

  camera(
      vector3f origin = 0.0f,
      vector3f x = { 1.0f, 0.0f, 0.0f }, 
      vector3f y = { 0.0f, 1.0f, 0.0f }) {
    basis.origin = origin;
    basis.x = x;
    basis.y = y;
    vector3f z = cross(x, y);
    basis.z = z/abs(z);
  }
  
  // This is templatized to enable computation of Jacobians when necessary.
  template <typename T>
  vector2<T> project(const vector3<T> &g) const {
    // Convert the global coordinates to the local basis.
    vector3<T> l = basis.local(g);

    // Project the local coordinates.
    T inv_z = rcp(l.z);
    return vector2<T>(l.x*inv_z, l.y*inv_z);
  }
};

#endif