#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <iomanip>

// Defines compile time constant dimension 2D matrices.

// A constant reference to a matrix. This is intended to be passed by value,
// as it is itself a reference. Copies are shallow.
template <typename T, int M, int N>
class const_matrix_ref {
protected:
  T *x;

public:
  const_matrix_ref(const T *x) : x(const_cast<T*>(x)) {}
  
  const T *elements() const { return x; }
  T at(int i, int j) const { return x[i*N + j]; }
  T operator() (int i, int j) const { return at(i, j); }
  T operator() (int i) const { 
    static_assert(M == 1 || N == 1, "matrix is not a vector.");
    if (N == 1)
      return at(i, 0); 
    else
      return at(0, i);
  }
};

// A non-constant reference to a matrix. This is intended to be passed by value,
// as it is itself a reference. Copies are shallow.
template <typename T, int M, int N>
class matrix_ref : public const_matrix_ref<T, M, N> {
public:
  typedef const_matrix_ref<T, M, N> const_ref;

protected:
  using const_ref::x;

public:
  matrix_ref(T *x) : const_ref(x) {}

  using const_ref::elements;
  using const_ref::at;
  using const_ref::operator();

  T *elements() { return x; }
  T &at(int i, int j) { return x[i*N + j]; }
  T &operator() (int i, int j) { return at(i, j); }
  T &operator() (int i) { 
    static_assert(M == 1 || N == 1, "matrix is not a vector.");
    if (N == 1)
      return at(i, 0); 
    else
      return at(0, i);
  }
  
  matrix_ref &operator += (matrix_ref B) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) += B(i, j);
    return *this;
  }

  matrix_ref &operator -= (matrix_ref B) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) -= B(i, j);
    return *this;
  }
  
  matrix_ref &operator *= (T b) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) *= b;
    return *this;
  }

  matrix_ref &operator /= (T b) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) /= b;
    return *this;
  }
};

// A non-reference matrix type. This class contains storage, and copies
// are deep.
template <typename T, int M, int N>
class matrix : public matrix_ref<T, M, N> {
  T storage[M*N];

public:
  typedef matrix_ref<T, M, N> ref;
  
  using ref::elements;
  using ref::at;
  using ref::operator();
  using ref::operator+=;
  using ref::operator-=;
  using ref::operator*=;
  using ref::operator/=;

  matrix() : ref(storage) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) = 0;
  }

  matrix(const matrix &A) : ref(storage) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) = A(i, j);
  }

  matrix(const std::initializer_list<std::initializer_list<T>> &rows) : ref(storage) {
    typename std::initializer_list<std::initializer_list<T>>::iterator r = rows.begin();
    for (int i = 0; i < M; i++, r++) {
      typename std::initializer_list<T>::iterator c = r->begin();
      for (int j = 0; j < N; j++)
        at(i, j) = *c++;
    }
  }

  matrix &operator = (const matrix &A) {
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) = A(i, j);
    return *this;
  }
};

template <typename T, int M, int N>
std::ostream &operator << (std::ostream &os, const_matrix_ref<T, M, N> A) {
  os << '[' << std::endl;
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++ )
      os << std::setw(16) << A(i, j);
    os << std::endl;
  }
  os << ']' << std::endl;
  return os;
}

template <typename T, int M, int N>
matrix<T, M, N> operator +(const_matrix_ref<T, M, N> A, const_matrix_ref<T, M, N> B) {
  matrix<T, M, N> C;
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++)
      C(i, j) = A(i, j) + B(i, j);
  return C;
}

template <typename T, int M, int N>
matrix<T, M, N> operator -(const_matrix_ref<T, M, N> A, const_matrix_ref<T, M, N> B) {
  matrix<T, M, N> C;
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++)
      C(i, j) = A(i, j) - B(i, j);
  return C;
}

template <typename T, int M, int N, int K>
matrix<T, M, K> operator *(const_matrix_ref<T, M, N> A, const_matrix_ref<T, N, K> B) {
  matrix<T, M, K> C;
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < K; j++) {
      T ij = 0;
      for (int k = 0; k < N; k++)
        ij += A(i, k)*B(k, j);
      C(i, j) = ij;
    }
  }
  return C;
}

template <typename T, int M, int N>
matrix<T, M, N> operator *(T a, const_matrix_ref<T, M, N> B) {
  matrix<T, M, N> C;
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++)
      C(i, j) = a*B(i, j);
  return C;
}

template <typename T, int M, int N>
matrix<T, M, N> operator *(const_matrix_ref<T, M, N> A, T b) {
  matrix<T, M, N> C;
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++)
      C(i, j) = A(i, j)*b;
  return C;
}

template <typename T, int N>
T dot(const_matrix_ref<T, N, 1> A, const_matrix_ref<T, N, 1> B) {
  const_matrix_ref<T, 1, N> AT(A.elements());
  return (AT*B)(0, 0);
}

// Solve A*x = b, using Gaussian elimination.
template <typename T, int N>
matrix_ref<T, N, 1> solve(matrix_ref<T, N, N> A, matrix_ref<T, N, 1> b) {
  // Set this to 0 to zero out the lower triangular portion of the matrix.
  // This isn't necessary to solve for x, so setting it to 1 can save
  // some FLOPs.
  const int implicit_zero = 1;

  for (int i = 0; i < N; ++i) {
    // Find a pivot row for this variable.
    int pi = i;
    T max = std::abs(A(i, i));
    for (int i2 = i + 1; i2 < N; ++i2) {
      T absi2j = std::abs(A(i2, i));
      if (absi2j > max) {
        pi = i2;
        max = absi2j;
      }
    }
    
    // Swap pivot row with the current row.
    if (pi != i) {
      for (int j = i; j < N; ++j)
        std::swap(A(i, j), A(pi, j));
      std::swap(b(i), b(pi));
    }
  
    // Eliminate the rows after the pivot.
    T p = A(i, i);
    //if (abs(p) < 1e-12)
    //  throw std::runtime_error("signular matrix.");
    for (int i2 = i + 1; i2 < N; ++i2) {
      T s = A(i2, i)/p;
      for (int j = i + implicit_zero; j < N; ++j)
        A(i2, j) -= A(i, j)*s;
      b(i2) -= b(i)*s;
    }
  }

  // A is now upper triangular, so we can solve it.
  for (int i = N - 1; i >= 0; i--) {
    T r = b(i);
    for (int j = i + 1; j < N; j++)
      r -= A(i, j)*b(j);
    b(i) = r/A(i, i);
  }
  
  return b;
}

template <typename T, int M, int N>
bool isnan(const_matrix_ref<T, M, N> A) {
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++)
      if (std::isnan(A(i, j)))
        return true;
  return false;
}

#endif