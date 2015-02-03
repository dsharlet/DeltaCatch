#ifndef EV3CV_MATH_VECTOR2_H
#define EV3CV_MATH_VECTOR2_H

namespace ev3cv {

template <typename T>
class vector2 {
public:
  T x, y;

  vector2(T c = 0) : x(c), y(c) {}
  vector2(T x, T y) : x(x), y(y) {}

  template <typename U> vector2<T> &operator += (const vector2<U> &r) { x += r.x; y += r.y; return *this; }
  template <typename U> vector2<T> &operator -= (const vector2<U> &r) { x -= r.x; y -= r.y; return *this; }
  template <typename U> vector2<T> &operator *= (U r) { x *= r; y *= r; return *this; }
  template <typename U> vector2<T> &operator /= (U r) { x /= r; y /= r; return *this; }

  // Vector products are pointwise.
  template <typename U> vector2<T> &operator *= (const vector2<U> &r) { x *= r.x; y *= r.y; return *this; }
  template <typename U> vector2<T> &operator /= (const vector2<U> &r) { x /= r.x; y /= r.y; return *this; }

  vector2<T> operator - () const { return vector2<T>(-x, -y); }
};

template <typename T, typename U> vector2<T> operator + (vector2<T> l, const vector2<U> &r) { return l += r; }
template <typename T, typename U> vector2<T> operator - (vector2<T> l, const vector2<U> &r) { return l -= r; }
template <typename T, typename U> vector2<T> operator * (vector2<T> l, U r) { return l *= r; }
template <typename T, typename U> vector2<U> operator * (T l, vector2<U> r) { return r *= l; }
template <typename T, typename U> vector2<T> operator / (vector2<T> l, U r) { return l /= r; }
template <typename T, typename U> vector2<T> operator * (vector2<T> l, const vector2<U> &r) { return l *= r; }
template <typename T, typename U> vector2<T> operator / (vector2<T> l, const vector2<U> &r) { return l /= r; }

template <typename T, typename U> 
T dot(const vector2<T> &l, const vector2<U> &r) { 
  return l.x*r.x + l.y*r.y;
}

template <typename T>
T abs(const vector2<T> &x) {
  return sqrt(dot(x, x));
}

template <typename T>
T sqr_abs(const vector2<T> &x) {
  return dot(x, x);
}

template <typename T>
vector2<T> unit(const vector2<T> &x) {
  return x/abs(x);
}

template <typename T>
vector2<T> min(const vector2<T> &a, const vector2<T> &b) {
  return vector2<T>(
    min(a.x, b.x),
    min(a.y, b.y));
}

template <typename T>
vector2<T> max(const vector2<T> &a, const vector2<T> &b) {
  return vector2<T>(
    max(a.x, b.x),
    max(a.y, b.y));
}

template <typename T>
bool isnan(const vector2<T> &x) { return isnan(x.x) || isnan(x.y); }

template <typename T>
std::ostream &operator << (std::ostream &os, const vector2<T> &x) {
  return os << '[' << x.x << ' ' << x.y << ']';
}

template <typename T>
std::istream &operator >> (std::istream &is, vector2<T> &x) {
  is.ignore(std::numeric_limits<std::streamsize>::max(), '[');
  is >> x.x >> x.y;
  is.ignore(std::numeric_limits<std::streamsize>::max(), ']');
  return is;
}

template <typename T, typename U>
vector2<T> vector_cast(const vector2<U> &x) {
  return vector2<T>(scalar_cast<T>(x.x), scalar_cast<T>(x.y));
}

typedef vector2<float> vector2f;
typedef vector2<double> vector2d;
typedef vector2<int> vector2i;

inline vector2f randv2f(const vector2f &a = 0.0f, const vector2f &b = 1.0f) {
  return vector2f(randf(a.x, b.x), randf(a.y, b.y));
}

}  // namespace ev3cv

#endif