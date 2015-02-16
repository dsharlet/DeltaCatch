/** \file quaternion.h
 * Defines quaternion type and helper functions.
 */

#ifndef EV3CV_MATH_QUATERNION_H
#define EV3CV_MATH_QUATERNION_H

namespace ev3cv {

/** Defines a quaternion \f$q = a + \mathbf{i} b_x + \mathbf{j} b_y + \mathbf{k} b_z\f$. */
template <typename T>
class quaternion {
public:
  T a;
  vector3<T> b;

  /** Construct a quaternion \f$q = a\f$. */
  quaternion(T a = 0) : a(a) {}
  /** Construct a quaternion \f$q = a + \mathbf{i} b + \mathbf{j} c + \mathbf{k} d\f$. */
  quaternion(T a, T b, T c, T d) : a(a), b(b, c, d) {}
  /** Construct a quaternion \f$q = a + \mathbf{i} b + \mathbf{j} c + \mathbf{k} d\f$. */
  quaternion(T a, const vector3<T> &b) : a(a), b(b) {}
  
  /** Construct a quaternion from an orthonormal basis \f$[x\;y\;z]\f$. */
  static quaternion<T> from_basis(const vector3<T> &x, const vector3<T> &y, const vector3<T> &z) {
    T tr = x.x + y.y + z.z;
    if (tr > 0) { 
      T S = sqrt(tr + 1.0) * 2;
      return quaternion<T>(
          0.25 * S,
          (y.z - z.y) / S,
          (z.x - x.z) / S,
          (x.y - y.x) / S);
    } else if ((x.x > y.y) && (x.x > z.z)) {
      T S = sqrt(1.0 + x.x - y.y - z.z) * 2;
      return quaternion<T>(
          (y.z - z.y) / S,
          0.25 * S,
          (y.x + x.y) / S,
          (z.x + x.z) / S);
    } else if (y.y > z.z) {
      T S = sqrt(1.0 + y.y - x.x - z.z) * 2;
      return quaternion<T>(
          (z.x - x.z) / S,
          (y.x + x.y) / S, 
          0.25 * S,
          (z.y + y.z) / S);
    } else {
      T S = sqrt(1.0 + z.z - x.x - y.y) * 2;
      return quaternion<T>(
          (x.y - y.x) / S,
          (z.x + x.z) / S,
          (z.y + y.z) / S,
          0.25 * S);
    }
  }

  /** Construct a quaternion from an orthonormal basis \f$[x\;y\;z]\f$, where \f$z=x \times y\f$.  */
  static quaternion<T> from_basis(const vector3<T> &x, const vector3<T> &y) {
    return from_basis(x, y, cross(x, y));
  }

  /** Arithmetic-assignment operators. */
  ///@{
  template <typename U>
  quaternion<T> &operator += (const quaternion<U> &r) { a += r.a, b += r.b; return *this; }
  template <typename U>
  quaternion<T> &operator -= (const quaternion<U> &r) { a -= r.a, b -= r.b; return *this; }
  template <typename U>
  quaternion<T> &operator *= (const quaternion<U> &r) { 
    T a_ = a*r.a - dot(b, r.b);
    b = r.b*a + r.a*b + cross(b, r.b);
    a = a_;
    return *this;
  }
  
  template <typename U>
  quaternion<T> &operator *= (const U &r) {
    a *= r;
    b *= r;
    return *this;
  }
  template <typename U>
  quaternion<T> &operator /= (const U &r) {
    a /= r;
    b /= r;
    return *this;
  }
  ///@}

  /** Negate the quaternion. */
  quaternion<T> operator - () const { return quaternion<T>(-a, -b); }

  /** Compute the conjugate of the quaternion. */
  quaternion<T> operator ~ () const { return quaternion<T>(a, -b); }
};

/** Basic quaternion arithmetic operators. */
///@{
template <typename T, typename U> quaternion<T> operator + (quaternion<T> l, const quaternion<U> &r) { return l += r; }
template <typename T, typename U> quaternion<T> operator - (quaternion<T> l, const quaternion<U> &r) { return l -= r; }
template <typename T, typename U> quaternion<T> operator * (quaternion<T> l, const quaternion<U> &r) { return l *= r; }
template <typename T, typename U> quaternion<T> operator * (quaternion<T> l, const U &r) { return l *= r; }
template <typename T, typename U> quaternion<U> operator * (const T &l, quaternion<U> r) { return r *= l; }
template <typename T, typename U> quaternion<T> operator / (quaternion<T> l, const U &r) { return l /= r; }

template <typename T, typename U> quaternion<T> operator * (quaternion<T> l, const vector3<U> &r) { return l *= quaternion<U>(0, r); }
template <typename T, typename U> quaternion<T> operator * (const vector3<T> &l, const quaternion<U> &r) { return quaternion<T>(0, l)*r; }
///@}

/** Compute the magnitude of a quaternion. */
template <typename T>
T abs(const quaternion<T> &q) {
  return sqrt(q.a*q.a + dot(q.b, q.b));
}

/** Compute a quaternion of equal direction with \f$|q|=1\f$. */
template <typename T>
quaternion<T> unit(const quaternion<T> &x) {
  return x/abs(x);
}

/** Check if any quaternion element is NaN. */
template <typename T>
bool isnan(const quaternion<T> &q) { return isnan(q.a) || isnan(q.b); }

/** Check if all quaternion elements are finite (not NaN or infinity). */
template <typename T>
bool isfinite(const quaternion<T> &q) { return isfinite(q.a) && isfinite(q.b); }

template <typename T, typename U>
quaternion<T> quaternion_cast(const quaternion<U> &q) {
  return quaternion<T>(scalar_cast<T>(q.a), vector_cast<T>(q.b));
}

template <typename T>
std::ostream &operator << (std::ostream &os, const quaternion<T> &q) {
  return os << '[' << q.a << ' ' << q.b.x << ' ' << q.b.y << ' ' << q.b.z << ']';
}

template <typename T>
std::istream &operator >> (std::istream &is, quaternion<T> &q) {
  is.ignore(std::numeric_limits<std::streamsize>::max(), '[');
  is >> q.a >> q.b.x >> q.b.y >> q.b.z;
  is.ignore(std::numeric_limits<std::streamsize>::max(), ']');
  return is;
}

typedef quaternion<float> quaternionf;

}  // namespace ev3cv

#endif