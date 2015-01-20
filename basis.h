#ifndef BASIS_H
#define BASIS_H

#include "vector2.h"
#include "vector3.h"
#include "quaternion.h"

// Basis of a 2D coordinate system.
template <typename T>
struct basis2 {
  vector2<T> x, y;
  vector2<T> origin;

  basis2(
      const vector2<T> &x = vector2<T>(1, 0),
      const vector2<T> &y = vector2<T>(0, 1),
      const vector2<T> &origin = vector2<T>(0)) : x(x), y(y), origin(origin) {}

  // Convert global coordinates to the local coordinates of this basis.
  template <typename U>
  vector2<U> to_local(vector2<U> g) const {
    g -= origin;
    return vector2<U>(dot(g, x), dot(g, y));
  }

  // Convert local basis coordinates to global coorinates.
  template <typename U>
  vector2<U> to_global(const vector2<U> &l) const {
    return l.x*x + l.y*y + origin;
  }
};

// Basis of a 3D coordinate system.
template <typename T>
struct basis3 {
  vector3<T> x, y, z;
  vector3<T> origin;

  basis3(
      const vector3<T> &x = vector3<T>(1, 0, 0),
      const vector3<T> &y = vector3<T>(0, 1, 0),
      const vector3<T> &z = vector3<T>(0, 0, 1),
      const vector3<T> &origin = vector3<T>(0)) : x(x), y(y), z(z), origin(origin) {}
  basis3(
      const quaternion<T> &q,
      const vector3<T> &origin = vector3<T>(0)) : origin(origin) {
    static quaternion<T> X(0, vector3<T>(1, 0, 0));
    static quaternion<T> Y(0, vector3<T>(0, 1, 0));
    static quaternion<T> Z(0, vector3<T>(0, 0, 1));
    
    x = (q * X * ~q).b;
    y = (q * Y * ~q).b;
    z = (q * Z * ~q).b;
  }

  // Convert global coordinates to the local coordinates of this basis.
  template <typename U>
  vector3<U> to_local(vector3<U> g) const {
    g -= origin;
    return vector3<U>(dot(g, x), dot(g, y), dot(g, z));
  }

  // Convert local basis coordinates to global coorinates.
  template <typename U>
  vector3<U> to_global(const vector3<U> &l) const {
    return l.x*x + l.y*y + l.z*z + origin;
  }
};

template <typename T, typename U>
basis2<T> basis_cast(const basis2<U> &B) {
  return basis2<T>(
      vector_cast<T>(B.x),
      vector_cast<T>(B.y),
      vector_cast<T>(B.origin));
}

template <typename T, typename U>
basis3<T> basis_cast(const basis3<U> &B) {
  return basis3<T>(
      vector_cast<T>(B.x),
      vector_cast<T>(B.y),
      vector_cast<T>(B.z),
      vector_cast<T>(B.origin));
}

typedef basis2<float> basis2f;
typedef basis3<float> basis3f;

#endif