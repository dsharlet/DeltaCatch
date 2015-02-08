/*! \mainpage ev3cv documentation
 *
 * \section intro_sec Introduction
 *
 * This is the introduction.
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step 1: Opening the box
 *  
 * etc...
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
using std::isnan;
using std::isinf;
using std::isfinite;
using std::min;
using std::max;
using std::sin;
using std::sqrt;
using std::tan;

static const float pi = 3.1415926535897f;

/** Compute the square of x, x^2. */
template <typename T> T sqr(T x) { return x*x; }
/** Compute the reciprocal of x, 1/x. */
template <typename T> T rcp(T x) { return 1/x; }

/** Bound x to be in the range [a, b]. */
template <typename T> T clamp(T x, T a, T b) { return min(max(x, a), b); }

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