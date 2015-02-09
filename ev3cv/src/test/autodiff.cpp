#include <ev3cv.h>
#include "test.h"

using namespace std;
using namespace ev3cv;

template <typename T> T f1_x(T x) { return x; }
template <typename T> T f1_Ax(T x) { return T(3)*x; }
template <typename T> T f1_linear(T x) { return T(3)*x + T(1); }
template <typename T> T f1_quadratic1(T x) { return T(-2)*x*x + T(8.5)*x + T(2); }
template <typename T> T f1_quadratic2(T x) { return T(-2)*sqr(x) + T(8.5)*x + T(2); }

template <typename T> T f1_1(T x) { return T(1.5)/x; }
template <typename T> T f1_2(T x) { return T(1)/x; }
template <typename T> T f1_3(T x) { return sqr(x); }
template <typename T> T f1_4(T x) { return sqrt(x); }
template <typename T> T f1_5(T x) { return sqrt(T(1.3)/sqrt(x)); }
template <typename T> T f1_6(T x) { return sqr(T(1)/sqrt(x)); }
template <typename T> T f1_7(T x) { return abs(x); }
template <typename T> T f1_8(T x) { return sqrt(abs(x*x*x)); }
template <typename T> T f1_9(T x) { return rcp(x); }
template <typename T> T f1_10(T x) { return sin(x); }
template <typename T> T f1_11(T x) { return cos(x); }
template <typename T> T f1_12(T x) { return asin(x); }
template <typename T> T f1_13(T x) { return acos(x); }

template <typename T> T f2_1(T x, T y) { return x/y; }
template <typename T> T f2_2(T x, T y) { return sqrt(sqr(x) + sqr(y)); }
template <typename T> T f2_3(T x, T y) { return T(1)/(sqr(x) + sqr(y)); }
template <typename T> T f2_4(T x, T y) { return T(1)/sqrt(sqr(x) + sqr(y)); }

const double epsilon = 1e-6;
const double h = 1e-6;

#define TEST_1T(T, fn, min, max, step) { \
  typedef diff<T, 1> d; \
  for (T x = min; x <= max; x += step) { \
    d fx = fn(d(x, 0)); \
    ASSERT_LT(abs(fx.f - fn(x)), epsilon); \
    ASSERT_LT(abs(fx.d(0) - (fn<double>(x + h) - fn<double>(x - h))/(2*h)), epsilon); \
  } \
}

#define TEST_2T(T, fn, minx, maxx, stepx, miny, maxy, stepy) { \
  typedef diff<T, 2> d; \
  for (T y = miny; y <= maxy; y += stepy) { \
    for (T x = minx; x <= maxx; x += stepx) { \
      d fxy = fn(d(x, 0), d(y, 1)); \
      ASSERT_LT(abs(fxy.f - fn(x, y)), epsilon); \
      ASSERT_LT(abs(fxy.d(0) - (fn<double>(x + h, y) - fn<double>(x - h, y))/(2*h)), epsilon); \
      ASSERT_LT(abs(fxy.d(1) - (fn<double>(x, y + h) - fn<double>(x, y - h))/(2*h)), epsilon); \
    } \
  } \
}

#define TEST_1(f, min, max, step) \
  TEST_1T(float, f, min, max, step); \
  TEST_1T(double, f, min, max, step);

#define TEST_2(f, minx, maxx, stepx, miny, maxy, stepy) \
  TEST_2T(float, f, minx, maxx, stepx, miny, maxy, stepy); \
  TEST_2T(double, f, minx, maxx, stepx, miny, maxy, stepy);

int main(int argc, const char **argv) {
  TEST_1(f1_x, -9.5, 9.5, 1);
  TEST_1(f1_Ax, -9.5, 9.5, 1);
  TEST_1(f1_linear, -9.5, 9.5, 1);
  TEST_1(f1_quadratic1, -9.5, 9.5, 1);
  TEST_1(f1_quadratic2, -9.5, 9.5, 1);
  TEST_1(f1_1, -9.5, 9.5, 1);
  TEST_1(f1_2, -9.5, 9.5, 1);
  TEST_1(f1_3, 0.5, 9.5, 1);
  TEST_1(f1_4, 0.5, 9.5, 1);
  TEST_1(f1_5, 0.5, 9.5, 1);
  TEST_1(f1_6, 0.5, 9.5, 1);
  TEST_1(f1_7, -9.5, 9.5, 1);
  TEST_1(f1_8, -9.5, 9.5, 1);
  TEST_1(f1_9, -9.5, 9.5, 1);
  TEST_1(f1_10, -9.5, 9.5, 1);
  TEST_1(f1_11, -9.5, 9.5, 1);
  TEST_1(f1_12, -0.9, 0.9, 0.2);
  TEST_1(f1_13, -0.9, 0.9, 0.2);

  TEST_2(f2_1, -9.5, 9.5, 1, -9.5, 9.5, 1);
  TEST_2(f2_2, -9.5, 9.5, 1, -9.5, 9.5, 1);
  TEST_2(f2_3, -9.5, 9.5, 1, -9.5, 9.5, 1);
  TEST_2(f2_4, -9.5, 9.5, 1, -9.5, 9.5, 1);
  return 0;
}