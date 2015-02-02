#ifndef EV3CV_MATH_VECTOR3_H
#define EV3CV_MATH_VECTOR3_H

namespace ev3cv {

template <typename T>
class vector3 {
public:
  T x, y, z;

  vector3(T c = 0) : x(c), y(c), z(c) {}
  vector3(T x, T y, T z) : x(x), y(y), z(z) {}

  template <typename U> vector3<T> &operator += (const vector3<U> &r) { x += r.x; y += r.y; z += r.z; return *this; }
  template <typename U> vector3<T> &operator -= (const vector3<U> &r) { x -= r.x; y -= r.y; z -= r.z; return *this; }
  template <typename U> vector3<T> &operator *= (U r) { x *= r; y *= r; z *= r; return *this; }
  template <typename U> vector3<T> &operator /= (U r) { x /= r; y /= r; z /= r; return *this; }

  // Vector products are pointwise.
  template <typename U> vector3<T> &operator *= (const vector3<U> &r) { x *= r.x; y *= r.y; z *= r.z; return *this; }
  template <typename U> vector3<T> &operator /= (const vector3<U> &r) { x /= r.x; y /= r.y; z /= r.z; return *this; }

  vector3<T> operator - () const { return vector3<T>(-x, -y, -z); }
};

template <typename T, typename U> vector3<T> operator + (vector3<T> l, const vector3<U> &r) { return l += r; }
template <typename T, typename U> vector3<T> operator - (vector3<T> l, const vector3<U> &r) { return l -= r; }
template <typename T, typename U> vector3<T> operator * (vector3<T> l, U r) { return l *= r; }
template <typename T, typename U> vector3<U> operator * (T l, vector3<U> r) { return r *= l; }
template <typename T, typename U> vector3<T> operator / (vector3<T> l, U r) { return l /= r; }
template <typename T, typename U> vector3<T> operator * (vector3<T> l, const vector3<U> &r) { return l *= r; }
template <typename T, typename U> vector3<T> operator / (vector3<T> l, const vector3<U> &r) { return l /= r; }

template <typename T, typename U> 
T dot(const vector3<T> &l, const vector3<U> &r) { 
  return l.x*r.x + l.y*r.y + l.z*r.z;
}

template <typename T>
vector3<T> cross(const vector3<T> &l, const vector3<T> &r) {
  return vector3<T>(
    l.y*r.z - l.z*r.y,
    l.z*r.x - l.x*r.z,
    l.x*r.y - l.y*r.x);
}

template <typename T>
T abs(const vector3<T> &x) {
  return sqrt(dot(x, x));
}

template <typename T>
T sqr_abs(const vector3<T> &x) {
  return dot(x, x);
}

template <typename T>
vector3<T> unit(const vector3<T> &x) {
  return x/abs(x);
}

template <typename T>
bool isnan(const vector3<T> &x) { return isnan(x.x) || isnan(x.y) || isnan(x.z); }

template <typename T>
std::ostream &operator << (std::ostream &os, const vector3<T> &x) {
  return os << '[' << x.x << ' ' << x.y << ' ' << x.z << ']';
}

template <typename T>
std::istream &operator >> (std::istream &is, vector3<T> &x) {
  is.ignore(std::numeric_limits<std::streamsize>::max(), '[');
  is >> x.x >> x.y >> x.z;
  is.ignore(std::numeric_limits<std::streamsize>::max(), ']');
  return is;
}

template <typename T, typename U>
vector3<T> vector_cast(const vector3<U> &x) {
  return vector3<T>(scalar_cast<T>(x.x), scalar_cast<T>(x.y), scalar_cast<T>(x.z));
}

typedef vector3<float> vector3f;
typedef vector3<double> vector3d;
typedef vector3<int> vector3i;

inline vector3f randv3f(const vector3f &a = 0.0f, const vector3f &b = 1.0f) {
  return vector3f(randf(a.x, b.x), randf(a.y, b.y), randf(a.z, b.z));
}

}  // namespace ev3cv

#endif