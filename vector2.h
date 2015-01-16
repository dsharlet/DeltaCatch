#ifndef VECTOR2_H
#define VECTOR2_H

#include <iostream>
#include <cmath>

template <typename T>
class vector2 {
public:
  T x, y;

  vector2(T c = 0) : x(c), y(c) {}
  vector2(T x, T y) : x(x), y(y) {}

  template <typename U>
  vector2<T> &operator += (const vector2<U> &r) { x += r.x; y += r.y; return *this; }
  template <typename U>
  vector2<T> &operator -= (const vector2<U> &r) { x -= r.x; y -= r.y; return *this; }
  template <typename U>
  vector2<T> &operator *= (U r) { x *= r; y *= r; return *this; }
  template <typename U>
  vector2<T> &operator /= (U r) { x /= r; y /= r; return *this; }

  vector2<T> operator - () const { return vector2<T>(-x, -y); }
};

template <typename T, typename U> vector2<T> operator + (vector2<T> l, const vector2<U> &r) { return l += r; }
template <typename T, typename U> vector2<T> operator - (vector2<T> l, const vector2<U> &r) { return l -= r; }
template <typename T, typename U> vector2<T> operator * (vector2<T> l, U r) { return l *= r; }
template <typename T, typename U> vector2<U> operator * (T l, vector2<U> r) { return r *= l; }
template <typename T, typename U> vector2<T> operator / (vector2<T> l, U r) { return l /= r; }

template <typename T, typename U> 
T dot(const vector2<T> &l, const vector2<U> &r) { 
  return l.x*r.x + l.y*r.y;
}

template <typename T>
T abs(const vector2<T> &x) {
  return std::sqrt(dot(x, x));
}

template <typename T>
bool isnan(const vector2<T> &x) { return std::isnan(x.x) || std::isnan(x.y); }

template <typename T>
std::ostream &operator << (std::ostream &os, const vector2<T> &x) {
  return os << '<' << x.x << ", " << x.y << '>';
}

template <typename T>
std::istream &operator >> (std::istream &is, vector2<T> &x) {
  if (is.peek() == '<') is.ignore();
  is >> x.x;
  if (is.peek() == ',') is.ignore();
  is >> x.y;
  if (is.peek() == '>') is.ignore();
  return is;
}

typedef vector2<float> vector2f;
typedef vector2<int> vector2i;

#endif