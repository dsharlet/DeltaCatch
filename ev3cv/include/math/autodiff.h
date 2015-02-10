#ifndef EV3CV_MATH_AUTODIFF_H
#define EV3CV_MATH_AUTODIFF_H

namespace ev3cv {

/** Implements the forward automatic differentiation scheme via operator overloading.
 * This implementation tracks N derivatives simultaneously. */
template <typename T, int N>
class diff {
  std::array<T, N> df_;

public:
  T f;

  /** Retrieve one of the derivatives associated with this value. */
  // @{
  T &d(int i) { return df_[i]; }
  T d(int i) const { return df_[i]; }
  // @}
  /** The number of derivatives associated with this value. */
  int n() const { return static_cast<int>(df_.size()); }

  /** Initialize a constant (all derivatives are zero). */
  diff(T f = 0) : f(f) { for (T& i : df_) i = 0; }
  /** Initialize a variable to a value. */
  diff(T f, int i) : f(f) { for (T& i : df_) i = 0; d(i) = 1; }

  diff &operator += (const diff &r) { 
    f += r.f; 
    for (int i = 0; i < n(); i++)
      d(i) += r.d(i);
    return *this;
  }
  diff &operator -= (const diff &r) { 
    f -= r.f; 
    for (int i = 0; i < n(); i++)
      d(i) -= r.d(i);
    return *this;
  }
  diff &operator *= (const diff &r) {
    for (int i = 0; i < n(); i++)
      d(i) = f*r.d(i) + d(i)*r.f;
    f *= r.f;
    return *this;
  }
  diff &operator /= (const diff &r) {
    T inv_rx2 = 1/(r.f*r.f);
    for (int i = 0; i < n(); i++)
      d(i) = (d(i)*r.f - f*r.d(i))*inv_rx2;
    f /= r.f;
    return *this;
  }
  diff &operator += (T r) { f += r; return *this; }
  diff &operator -= (T r) { f -= r; return *this; }
  diff &operator *= (T r) {
    for (int i = 0; i < n(); i++)
      d(i) *= r;
    f *= r;
    return *this;
  }
  diff &operator /= (T r) {
    for (int i = 0; i < n(); i++)
      d(i) /= r;
    f /= r;
    return *this;
  }
  diff operator -() const {
    diff r(-f);
    for (int i = 0; i < n(); i++)
      r.d(i) = -d(i);
    return r;
  }
};

template <typename T, int N>
T &D(diff<T, N> &x, int i) { return x.d(i); }
template <typename T, int N>
T D(const diff<T, N> &x, int i) { return x.d(i); }

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

template <typename T, int N> bool operator <  (const diff<T, N> &l, const diff<T, N> &r) { return l.f <  r.f; }
template <typename T, int N> bool operator <= (const diff<T, N> &l, const diff<T, N> &r) { return l.f <= r.f; }
template <typename T, int N> bool operator >  (const diff<T, N> &l, const diff<T, N> &r) { return l.f >  r.f; }
template <typename T, int N> bool operator >= (const diff<T, N> &l, const diff<T, N> &r) { return l.f >= r.f; }
template <typename T, int N> bool operator == (const diff<T, N> &l, const diff<T, N> &r) { return l.f == r.f; }
template <typename T, int N> bool operator != (const diff<T, N> &l, const diff<T, N> &r) { return l.f != r.f; }

template <typename T, int N> bool operator <  (const diff<T, N> &l, T r) { return l.f <  r; }
template <typename T, int N> bool operator <= (const diff<T, N> &l, T r) { return l.f <= r; }
template <typename T, int N> bool operator >  (const diff<T, N> &l, T r) { return l.f >  r; }
template <typename T, int N> bool operator >= (const diff<T, N> &l, T r) { return l.f >= r; }
template <typename T, int N> bool operator == (const diff<T, N> &l, T r) { return l.f == r; }
template <typename T, int N> bool operator != (const diff<T, N> &l, T r) { return l.f != r; }

template <typename T, int N> bool operator <  (T l, const diff<T, N> &r) { return l <  r.f; }
template <typename T, int N> bool operator <= (T l, const diff<T, N> &r) { return l <= r.f; }
template <typename T, int N> bool operator >  (T l, const diff<T, N> &r) { return l >  r.f; }
template <typename T, int N> bool operator >= (T l, const diff<T, N> &r) { return l >= r.f; }
template <typename T, int N> bool operator == (T l, const diff<T, N> &r) { return l == r.f; }
template <typename T, int N> bool operator != (T l, const diff<T, N> &r) { return l != r.f; }

template <typename T, int N>
diff<T, N> sqrt(const diff<T, N> &x) {
  diff<T, N> r(x);
  r.f = sqrt(r.f);
  T du = 0.5/r.f;
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  return r;
}

template <typename T, int N>
diff<T, N> sqr(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = 2*r.f;
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f *= r.f;
  return r;
}

template <typename T, int N>
diff<T, N> rcp(const diff<T, N> &x) {
  diff<T, N> r(x);
  T inv_rx2 = -1/(r.f*r.f);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= inv_rx2;
  r.f = 1/r.f;
  return r;
}

template <typename T, int N>
diff<T, N> abs(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = r.f < 0 ? -1 : 1;
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = abs(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> exp(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = exp(r.f);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = exp(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> log(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = rcp(r.f);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = log(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> sin(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = cos(r.f);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = sin(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> cos(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = -sin(r.f);
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = cos(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> asin(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = rcp(sqrt(1 - sqr(r.f)));
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = asin(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> acos(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = -rcp(sqrt(1 - sqr(r.f)));
  for (int i = 0; i < r.n(); i++)
    r.d(i) *= du;
  r.f = acos(r.f);
  return r;
}

template <typename T, int N>
diff<T, N> atan2(const diff<T, N> &y, const diff<T, N> &x) {
  diff<T, N> r(x);
  T dx = -y.f/(sqr(x.f) + sqr(y.f));
  T dy = x.f/(sqr(x.f) + sqr(y.f));
  for (int i = 0; i < r.n(); i++)
    r.d(i) = x.d(i)*dx + y.d(i)*dy;
  r.f = atan2(y.f, x.f);
  return r;
}

template <typename T, typename U, int N>
T scalar_cast(const diff<U, N> &x) {
  return scalar_cast<T>(x.f);
}

template <typename T, int N>
std::ostream &operator << (std::ostream &os, const diff<T, N> &d) {
  return os << d.f;
}

template <typename T, int N>
std::istream &operator >> (std::istream &is, diff<T, N> &d) {
  return is >> d.f;
}

template <typename T, int N>
bool isfinite(const diff<T, N> &x) {
  for (int i = 0; i < x.n(); i++)
    if (!isfinite(x.d(i))) 
      return false;
  return isfinite(x.f);
}

template <typename T, int N>
bool isnan(const diff<T, N> &x) {
  for (int i = 0; i < x.n(); i++)
    if (!isnan(x.d(i))) 
      return false;
  return isnan(x.f);
}

}  // namespace ev3cv

#endif