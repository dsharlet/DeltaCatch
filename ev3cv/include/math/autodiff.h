// Copyright 2015 Google, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \file autodiff.h
 * Defines automatic differentation types and functions.
 */

/** \page autodiff Automatic differentiation

Automatic differentiation (AD) is an alternative method of computing derivatives, alongside
numerical finite differences or symbolic differentiation. AD is more numerically stable and
efficient compared to finite differences, and is easier to implement and less intrusive to
algorithm code than symbolic differentiation (or manually finding derivatives of an expression).

AD works by computing derivatives at the same time as evaluating a function. An AD library
provides a set of basic arithmetic building blocks using the chain rule.

For a more thorough explanation of automatic differentiation, see
<a href="http://en.wikipedia.org/wiki/Automatic_differentiation">Automatic differentiation</a>
on wikipedia. The rest of this page describes the specifics of the ev3cv::diff type.

The typical technique to use this type is to define a function of which the derivative is needed
as a template, and call this function with this type. For example:

\code
// Evaluate the polynomial A*x^2 + B*x + C
template <typename T>
T f(T x) {
  return T(3)*sqr(x) + T(1)*x + T(4);
}
\endcode

Now, the derivative of f can be computed like so:
\code
diff<double, 1> x = 3.5;

// By default, diff types have derivatives of 0 (i.e. constants),
// but since this is x, it should have a derivative of 1.
D(x) = 1;

// Evaluate y = f(x).
diff<double, 1> y = f(x);

// The above line computed both f(3.5) and f'(3.5):
double fx = scalar_cast<double>(y);
double df_dx = D(y);
\endcode

The above example only computed a single derivative. However, ev3cv::diff supports computing
up to some fixed number of partial derivatives simultaneously:

\code
template <typename T>
T f(T x, T y) {
  return T(3)*sqr(x) + T(1)*x*y + T(4)*sqr(y);
}
\endcode

\code
diff<double, 2> x = 3.14159;
diff<double, 2> y = 2.71828;

// Note that the indices here correpsond to variables. x -> 0, y -> 1.
D(x, 0) = 1;
D(y, 1) = 1;

// Evaluate z = f(x, y).
diff<double, 1> z = f(x, y);

// The above line computed all of f(x, y), df/dx and df/dy at x, y.
double fxy = scalar_cast<double>(z);
double df_dx = D(z, 0);
double df_dy = D(z, 1);
\endcode

*/

#ifndef EV3CV_MATH_AUTODIFF_H
#define EV3CV_MATH_AUTODIFF_H

namespace ev3cv {

/** Implements the forward automatic differentiation scheme via operator overloading. For more information,
 * see \ref autodiff.
 **/
template <typename T, int N>
class diff {
  std::array<T, N> du_;

public:
  T u;

  /** The number of variables for which a derivative is available. */
  int n() const { return static_cast<int>(du_.size()); }

  /** Retrieve one of the derivatives associated with this value. */
  ///@{
  T &d(int i = 0) { assert(i < n()); return du_[i]; }
  T d(int i = 0) const { return i < n() ? du_[i] : 0; }
  ///@}

  /** Initialize a constant (all derivatives are zero). */
  diff(T u = 0) : u(u) { for (T& i : du_) i = 0; }
  /** Initialize a variable to a value. */
  diff(T u, int i) : u(u) { for (T& i : du_) i = 0; d(i) = 1; }

  diff &operator += (const diff &r) {
    u += r.u;
    for (int i = 0; i < n(); i++)
      d(i) += r.d(i);
    return *this;
  }
  diff &operator -= (const diff &r) {
    u -= r.u;
    for (int i = 0; i < n(); i++)
      d(i) -= r.d(i);
    return *this;
  }
  diff &operator *= (const diff &r) {
    for (int i = 0; i < n(); i++)
      d(i) = u*r.d(i) + d(i)*r.u;
    u *= r.u;
    return *this;
  }
  diff &operator /= (const diff &r) {
    T inv_rx2 = 1/(r.u*r.u);
    for (int i = 0; i < n(); i++)
      d(i) = (d(i)*r.u - u*r.d(i))*inv_rx2;
    u /= r.u;
    return *this;
  }
  diff &operator += (T r) { u += r; return *this; }
  diff &operator -= (T r) { u -= r; return *this; }
  diff &operator *= (T r) {
    for (int i = 0; i < n(); i++)
      d(i) *= r;
    u *= r;
    return *this;
  }
  diff &operator /= (T r) {
    for (int i = 0; i < n(); i++)
      d(i) /= r;
    u /= r;
    return *this;
  }
  diff operator -() const {
    diff r(-u);
    for (int i = 0; i < n(); i++)
      r.d(i) = -d(i);
    return r;
  }
};

/** Get or set the derivative of x with respect to the i'th variable. */
///@{
template <typename T, int N>
T &D(diff<T, N> &x, int i = 0) { return x.d(i); }
template <typename T, int N>
T D(const diff<T, N> &x, int i = 0) { return x.d(i); }
///@}

/** Define basic arithmetic and comparison for automatic differentiation types. */
///@{
template <typename T, int N> diff<T, N> operator + (diff<T, N> l, const diff<T, N> &r) { return l += r; }
template <typename T, int N> diff<T, N> operator - (diff<T, N> l, const diff<T, N> &r) { return l -= r; }
template <typename T, int N> diff<T, N> operator * (diff<T, N> l, const diff<T, N> &r) { return l *= r; }
template <typename T, int N> diff<T, N> operator / (diff<T, N> l, const diff<T, N> &r) { return l /= r; }

template <typename T, int N> diff<T, N> operator + (diff<T, N> l, T r) { return l += r; }
template <typename T, int N> diff<T, N> operator - (diff<T, N> l, T r) { return l -= r; }
template <typename T, int N> diff<T, N> operator * (diff<T, N> l, T r) { return l *= r; }
template <typename T, int N> diff<T, N> operator / (diff<T, N> l, T r) { return l /= r; }

template <typename T, int N> diff<T, N> operator + (T l, diff<T, N> r) { return r += l; }
template <typename T, int N> diff<T, N> operator * (T l, diff<T, N> r) { return r *= l; }
template <typename T, int N> diff<T, N> operator - (T l, const diff<T, N> &r) { return diff<T, N>(l) -= r; }
template <typename T, int N> diff<T, N> operator / (T l, const diff<T, N> &r) { return diff<T, N>(l) /= r; }

template <typename T, int N> bool operator <  (const diff<T, N> &l, const diff<T, N> &r) { return l.u <  r.u; }
template <typename T, int N> bool operator <= (const diff<T, N> &l, const diff<T, N> &r) { return l.u <= r.u; }
template <typename T, int N> bool operator >  (const diff<T, N> &l, const diff<T, N> &r) { return l.u >  r.u; }
template <typename T, int N> bool operator >= (const diff<T, N> &l, const diff<T, N> &r) { return l.u >= r.u; }
template <typename T, int N> bool operator == (const diff<T, N> &l, const diff<T, N> &r) { return l.u == r.u; }
template <typename T, int N> bool operator != (const diff<T, N> &l, const diff<T, N> &r) { return l.u != r.u; }

template <typename T, int N> bool operator <  (const diff<T, N> &l, T r) { return l.u <  r; }
template <typename T, int N> bool operator <= (const diff<T, N> &l, T r) { return l.u <= r; }
template <typename T, int N> bool operator >  (const diff<T, N> &l, T r) { return l.u >  r; }
template <typename T, int N> bool operator >= (const diff<T, N> &l, T r) { return l.u >= r; }
template <typename T, int N> bool operator == (const diff<T, N> &l, T r) { return l.u == r; }
template <typename T, int N> bool operator != (const diff<T, N> &l, T r) { return l.u != r; }

template <typename T, int N> bool operator <  (T l, const diff<T, N> &r) { return l <  r.u; }
template <typename T, int N> bool operator <= (T l, const diff<T, N> &r) { return l <= r.u; }
template <typename T, int N> bool operator >  (T l, const diff<T, N> &r) { return l >  r.u; }
template <typename T, int N> bool operator >= (T l, const diff<T, N> &r) { return l >= r.u; }
template <typename T, int N> bool operator == (T l, const diff<T, N> &r) { return l == r.u; }
template <typename T, int N> bool operator != (T l, const diff<T, N> &r) { return l != r.u; }
///@}

/** Define some standard math functions for automatic differentiation. */
///@{
template <typename T, int N>
diff<T, N> rcp(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = -1/(r.u*r.u);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = 1/r.u;
  return r;
}

template <typename T, int N>
diff<T, N> sqrt(const diff<T, N> &x) {
  diff<T, N> r(x);
  r.u = sqrt(r.u);
  T du = 0.5/r.u;
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  return r;
}

template <typename T, int N>
diff<T, N> sqr(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = 2*r.u;
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u *= r.u;
  return r;
}

template <typename T, int N>
diff<T, N> exp(const diff<T, N> &x) {
  diff<T, N> r(x);
  r.u = exp(r.u);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= r.u;
  return r;
}

template <typename T, int N>
diff<T, N> log(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = rcp(r.u);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = log(r.u);
  return r;
}

template <typename T, int N>
diff<T, N> sin(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = cos(r.u);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = sin(r.u);
  return r;
}

template <typename T, int N>
diff<T, N> cos(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = -sin(r.u);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = cos(r.u);
  return r;
}

template <typename T, int N>
diff<T, N> asin(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = rcp(sqrt(1 - sqr(r.u)));
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = asin(r.u);
  return r;
}

template <typename T, int N>
diff<T, N> acos(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = -rcp(sqrt(1 - sqr(r.u)));
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = acos(r.u);
  return r;
}

template <typename T, int N>
diff<T, N> atan2(const diff<T, N> &y, const diff<T, N> &x) {
  diff<T, N> r(x);
  T dx = -y.u/(sqr(x.u) + sqr(y.u));
  T dy = x.u/(sqr(x.u) + sqr(y.u));
  for (int i = 0; i < r.n(); i++)
    r.d(i) = x.d(i)*dx + y.d(i)*dy;
  r.u = atan2(y.u, x.u);
  return r;
}

template <typename T, int N>
diff<T, N> abs(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = r.u < 0 ? -1 : 1;
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.u = abs(r.u);
  return r;
}
///@}

template <typename T, typename U, int N>
T scalar_cast(const diff<U, N> &x) {
  return scalar_cast<T>(x.u);
}

template <typename T, int N>
std::ostream &operator << (std::ostream &os, const diff<T, N> &d) {
  return os << d.u;
}

template <typename T, int N>
std::istream &operator >> (std::istream &is, diff<T, N> &d) {
  return is >> d.u;
}

template <typename T, int N>
bool isfinite(const diff<T, N> &x) {
  for (int i = 0; i < x.n(); i++)
    if (!isfinite(x.d(i)))
      return false;
  return isfinite(x.u);
}

template <typename T, int N>
bool isnan(const diff<T, N> &x) {
  for (int i = 0; i < x.n(); i++)
    if (!isnan(x.d(i)))
      return false;
  return isnan(x.u);
}

}  // namespace ev3cv

#endif
