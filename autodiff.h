#ifndef AUTODIFF_H
#define AUTODIFF_H

// Implementation of automatic differentiation via operator overloading.
template <typename T, int N>
class diff {
public:
  T f;
  T df[N];

  diff(T f = 0) : f(f) { for (T& i : df) i = 0; }
  diff(T f, int n) : f(f) { for (T& i : df) i = 0; df[n] = 1.0f; }

  diff &operator += (const diff &r) { f += r.f; for (int i = 0; i < N; i++) df[i] += r.df[i]; return *this; }
  diff &operator -= (const diff &r) { f -= r.f; for (int i = 0; i < N; i++) df[i] -= r.df[i]; return *this; }
  diff &operator *= (const diff &r) {
    for (int i = 0; i < N; i++)
      df[i] = f*r.df[i] + df[i]*r.f;
    f *= r.f;
    return *this;
  }
  diff &operator /= (const diff &r) {
    T inv_rx2 = 1/(r.f*r.f);
    for (int i = 0; i < N; i++)
      df[i] = (df[i]*r.f - f*r.df[i])*inv_rx2;
    f /= r.f;
    return *this;
  }
  template <typename U>
  diff &operator += (U r) { f += r; return *this; }
  template <typename U>
  diff &operator -= (U r) { f -= r; return *this; }
  diff operator -() const {
    diff n(-f);
    for (int i = 0; i < N; i++)
      n.df[i] = -df[i];
    return n;
  }
};

template <typename T, int N>
T &D(diff<T, N> &x, int n) { return x.df[n]; }
template <typename T, int N>
T D(const diff<T, N> &x, int n) { return x.df[n]; }

template <typename T, int N> diff<T, N> operator + (diff<T, N> l, const diff<T, N> &r) { return l += r; }
template <typename T, int N> diff<T, N> operator - (diff<T, N> l, const diff<T, N> &r) { return l -= r; }
template <typename T, int N> diff<T, N> operator * (diff<T, N> l, const diff<T, N> &r) { return l *= r; }
template <typename T, int N> diff<T, N> operator / (diff<T, N> l, const diff<T, N> &r) { return l /= r; }

template <typename T, int N, typename U> diff<T, N> operator + (diff<T, N> l, U r) { return l += r; }
template <typename T, int N, typename U> diff<T, N> operator - (diff<T, N> l, U r) { return l -= r; }
template <typename T, int N, typename U> diff<T, N> operator * (diff<T, N> l, U r) { return l *= r; }
template <typename T, int N, typename U> diff<T, N> operator / (diff<T, N> l, U r) { return l /= r; }

template <typename T, int N, typename U> diff<T, N> operator + (U l, diff<T, N> r) { return r += l; }
template <typename T, int N, typename U> diff<T, N> operator * (U l, diff<T, N> r) { return r *= l; }
template <typename T, int N, typename U> diff<T, N> operator - (U l, const diff<T, N> &r) { return diff<T, N>(l) -= r; }
template <typename T, int N, typename U> diff<T, N> operator / (U l, const diff<T, N> &r) { return diff<T, N>(l) /= r; }

template <typename T, int N>
diff<T, N> sqrt(const diff<T, N> &x) {
  diff<T, N> r(x);
  r.f = sqrt(r.f);
  T du = 0.5/r.f;
  for (int i = 0; i < N; i++) {
    r.df[i] *= du;
  }
  return r;
}

template <typename T>
T sqr(T x) {
  return x*x;
}

template <typename T, int N>
diff<T, N> sqr(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = 2*r.f;
  for (int i = 0; i < N; i++)
    r.df[i] *= du;
  r.f *= r.f;
  return r;
}

template <typename T>
T rcp(T x) {
  return 1/x;
}

template <typename T, int N>
diff<T, N> rcp(const diff<T, N> &x) {
  diff<T, N> r(x);
  T inv_rx2 = -1/(r.f*r.f);
  for (int i = 0; i < N; i++)
    r.df[i] *= inv_rx2;
  r.f = 1/r.f;
  return r;
}

template <typename T, int N>
diff<T, N> abs(const diff<T, N> &x) {
  diff<T, N> r(x);
  T du = r.f < 0 ? -1 : 1;
  for (int i = 0; i < N; i++) {
    r.df[i] *= du;
  }
  r.f = abs(r.f);
  return r;
}

#endif