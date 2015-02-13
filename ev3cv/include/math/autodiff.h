#ifndef EV3CV_MATH_AUTODIFF_H
#define EV3CV_MATH_AUTODIFF_H

namespace ev3cv {

/** Implements the forward automatic differentiation scheme via operator overloading. This type
 * can be used to compute the derivative of many mathematical expressions.
 *
 * The typical technique to use this type is to define a function as a template, for example:
 *
 *    // Evaluate the polynomial A*x^2 + B*x + C
 *    template <typename T>
 *    T f(T A, T B, T C, T x) {
 *        return A*x*x + B*x + C;
 *    }
 *     
 * Now, the derivative of quadratic can be computed like so:
 *
 *    diff<double, 1> x = 3.5;
 *    
 *    // By default, diff types have derivatives of 0, but since 
 *    // this is x, it has a derivative of 1.
 *    D(x, 0) = 1.
 *    
 *    // Evaluate y = f(x).
 *    diff<double, 1> A = 2, B = 3, C = 1;
 *    diff<double, 1> y = f(A, B, C, x);
 *    
 *    // The above line also computed f'(x):
 *    double df_dx = D(y, 0);
 *
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
  T inv_rx2 = -1/(r.u*r.u);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= inv_rx2;
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