/** 
\mainpage ev3cv documentation

\section about About

ev3cv is a library for helping users of <a href="www.ev3dev.org">ev3dev</a> and cameras like 
<a href="http://www.mindsensors.com/index.php?module=pagemaster&PAGE_user_op=view_page&PAGE_id=78">NXTcam</a> to build
robots with LEGOs that have stereo machine vision.

ev3cv also has some other useful support libraries, such as motor controllers (ev3cv::servo) and low level support for 
NXTcam (ev3cv::nxtcam) in ev3dev.

Here are a few demo videos using ev3cv:

- <a href="https://www.youtube.com/watch?v=jHaB2zdfhHg">Stereo 3D tracking of a ball</a>
- <a href="https://www.youtube.com/watch?v=vxx3bBVWaxg">ev3dev::motor vs. ev3cv::servo</a>

All of the code in ev3cv is designed to run on the EV3 brick running ev3dev.  
*/

#ifndef EV3CV_EV3CV_H
#define EV3CV_EV3CV_H

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ev3cv {

using std::abs;
using std::acos;
using std::asin;
using std::atan;
using std::atan2;
using std::cos;
using std::exp;
using std::isnan;
using std::isinf;
using std::isfinite;
using std::log;
using std::min;
using std::max;
using std::sin;
using std::sqrt;
using std::tan;

static const float pi = 3.1415926535897f;

/** Compute the square of x, x^2. */
template <typename T> 
T sqr(T x) { return x*x; }
/** Compute the reciprocal of x, 1/x. */
template <typename T> 
T rcp(T x) { return 1/x; }

/** Bound x to be in the range [a, b]. */
template <typename T> 
T clamp(T x, T a, T b) { return min(max(x, a), b); }

/** Evaluate \f$sinc(x) = sin(x)/x\f$. */
template <typename T> 
T sinc(T x) { return abs(x) < 1e-6 ? 1 : sin(x)/x; }

/** Compute a random float in [a, b). */
inline float randf(float a = 0.0f, float b = 1.0f) { return (static_cast<float>(rand()) / RAND_MAX)*(b - a) + a; }

/** Cast a scalar value to the specified type. */
template <typename T, typename U>
T scalar_cast(const U &x) { return static_cast<T>(x); }

/** Return an instance of std::ostream that discards its output. */
std::ostream &null_ostream();

}  // namespace ev3cv

#include "circular_array.h"

#include "math/matrix.h"
#include "math/vector2.h"
#include "math/vector3.h"
#include "math/quaternion.h"
#include "math/autodiff.h"
#include "math/pid_controller.h"

using namespace ev3cv;

#endif