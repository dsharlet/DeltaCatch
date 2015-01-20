#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <cstdlib>

// TODO: Figure out how to conveniently overload math functions without doing this.
using namespace std;

static const float pi = 3.1415926535897f;

template <typename T> T sqr(T x) { return x*x; }
template <typename T> T rcp(T x) { return 1/x; }

inline float randf(float a = 0.0f, float b = 1.0f) { return (static_cast<float>(rand()) / RAND_MAX)*(b - a) + a; }

template <typename T, typename U>
T scalar_cast(const U &x) { return static_cast<T>(x); }

#endif