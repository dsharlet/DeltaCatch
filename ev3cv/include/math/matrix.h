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

/** \file matrix.h
 * Defines linear algebra types and helper functions.
 */

#ifndef EV3CV_MATH_MATRIX_H
#define EV3CV_MATH_MATRIX_H

#include <iomanip>
#include <initializer_list>
#include <vector>

namespace ev3cv {

/** A constant reference to a matrix. This is intended to be passed by value,
 * as it is itself a reference type, i.e. copies are shallow.
 * If the template dimensions are 0, the matrix is dynamically sized. */
template <typename T, int M_ = 0, int N_ = 0>
class const_matrix_ref {
protected:
  T *x;
  int m, n;

public:
  const_matrix_ref(const T *x, int M = 0, int N = 0) : x(const_cast<T*>(x)), m(M), n(N) {}

  /** Get the constant memory this matrix reference refers to. */
  const T *elements() const { return x; }

  /** Get the number of rows (M) and columns (N) in this matrix */
  ///@{
  inline int M() const { return M_ != 0 ? M_ : m; }
  inline int N() const { return N_ != 0 ? N_ : n; }
  ///@}

  /** Get an element at row i, column j. */
  ///@{
  T at(int i, int j) const { return x[i*N() + j]; }
  T operator() (int i, int j) const { return at(i, j); }
  ///@}

  /** Assuming this matrix is a vector (either M == 1 or N == 1), get the i'th element of the vector. */
  T operator() (int i) const {
    static_assert((M_ == 0 || M_ == 1) || (N_ == 0 || N_ == 1), "matrix is not a vector.");
    assert(M() == 1 || N() == 1);
    if (N() == 1)
      return at(i, 0);
    else
      return at(0, i);
  }
};

/** A non-constant reference to a matrix. This is intended to be passed by value,
 * as it is itself a reference type, i.e. copies are shallow.
 * If the template dimensions are 0, the matrix is dynamically sized. */
template <typename T, int M_ = 0, int N_ = 0>
class matrix_ref : public const_matrix_ref<T, M_, N_> {
public:
  typedef const_matrix_ref<T, M_, N_> const_ref;

protected:
  using const_ref::x;

public:
  matrix_ref(T *x, int M = 0, int N = 0) : const_ref(x, M, N) {}

  using const_ref::elements;
  using const_ref::at;
  using const_ref::operator();
  using const_ref::M;
  using const_ref::N;

  /** Get the non-const memory this matrix reference refers to. */
  T *elements() { return x; }

  /** Access an element at row i, column j. */
  ///@{
  T &at(int i, int j) { return x[i*N() + j]; }
  T &operator() (int i, int j) { return at(i, j); }
  ///@}

  /** Assuming this matrix is a vector (either M == 1 or N == 1), access the i'th element of the vector. */
  T &operator() (int i) {
    static_assert((M_ == 0 || M_ == 1) || (N_ == 0 || N_ == 1), "matrix is not a vector.");
    assert(M() == 1 || N() == 1);
    if (N() == 1)
      return at(i, 0);
    else
      return at(0, i);
  }

  /** Matrix arithmetic-assignment operators */
  ///@{
  matrix_ref &operator += (const_ref B) {
    assert(M() == B.M() && N() == B.N());
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) += B(i, j);
    return *this;
  }

  matrix_ref &operator -= (const_ref B) {
    assert(M() == B.M() && N() == B.N());
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) -= B(i, j);
    return *this;
  }

  matrix_ref &operator *= (T b) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) *= b;
    return *this;
  }

  matrix_ref &operator /= (T b) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) /= b;
    return *this;
  }
  ///@}

  /** Copy the contents of A to this matrix. */
  matrix_ref &operator = (const_ref A) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = A(i, j);
    return *this;
  }
};

/** Non-reference matrix type. This class contains storage, and copies
 * are deep. If the static dimensions M_ and N_ are not provided, the size
 * of the matrix is determined at runtime. */
template <typename T, int M_ = 0, int N_ = 0>
class matrix : public matrix_ref<T, M_, N_> {
public:
  typedef matrix_ref<T, M_, N_> ref;
  typedef const_matrix_ref<T, M_, N_> const_ref;
  using ref::M;
  using ref::N;

protected:
  T storage[M_*N_];

public:
  using ref::elements;
  using ref::at;
  using ref::operator();
  using ref::operator+=;
  using ref::operator-=;
  using ref::operator*=;
  using ref::operator/=;

  /** The default constructor zero-initializes the matrix. */
  matrix() : ref(storage) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = 0;
  }

  /** Zero initialize a matrix of a specific size. For statically sized matrices, the dimensions must match the static dimensions. */
  matrix(int M, int N) : matrix() {
    assert(M == M_ && N == N_);
  }

  /** Construct a copy of another matrix. */
  ///@{
  matrix(const_ref A) : ref(storage) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = A(i, j);
  }
  matrix(const matrix &A) : matrix(static_cast<const_ref>(A)) {}
  ///@}

  /** Construct a diagonal matrix of the value a. */
  matrix(const T &a) : ref(storage) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = (i == j) ? a : 0;
  }

  /** Initialize a matrix with an initializer list of initializer lists, for example:
   * \code
   * matrix<float, 3, 2> A = {
   *     { 1.0f, 2.0f },
   *     { 3.0f, 4.0f },
   *     { 5.0f, 6.0f }
   * };
   * \endcode
   * The inner initializer lists are the rows of the matrix. Elements not initialized are
   * initialized to 0.
   */
  matrix(const std::initializer_list<std::initializer_list<T>> &rows) : matrix() {
    assert(static_cast<int>(rows.size()) <= M());
    typename std::initializer_list<std::initializer_list<T>>::iterator r = rows.begin();
    for (int i = 0; i < M() && r != rows.end(); i++, r++) {
      assert(static_cast<int>(r->size()) <= N());
      typename std::initializer_list<T>::iterator c = r->begin();
      for (int j = 0; j < N() && c != r->end(); j++)
        at(i, j) = *c++;
    }
  }

  /** Copy the value of another matrix to this matrix. */
  ///@{
  matrix &operator = (const_ref A) {
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = A(i, j);
    return *this;
  }
  matrix &operator = (const matrix &A) { return *this = static_cast<const_ref>(A); }
  ///@}
};


/** A specialization of the 0x0 matrix. This holds a dynamically allocated matrix
  * instead of a static array. */
template <typename T>
class matrix<T, 0, 0> : public matrix_ref<T, 0, 0> {
public:
  typedef matrix_ref<T, 0, 0> ref;
  typedef const_matrix_ref<T, 0, 0> const_ref;
  using ref::M;
  using ref::N;

protected:
  std::vector<T> storage;

public:
  using ref::elements;
  using ref::at;
  using ref::operator();
  using ref::operator+=;
  using ref::operator-=;
  using ref::operator*=;
  using ref::operator/=;

  /** The default constructor constructs a zero-initialized 1x1 matrix. */
  matrix() : matrix(1, 1) {}

  matrix(const_ref A) : ref(nullptr, A.M(), A.N()), storage(A.M()*A.N()) {
    const_ref::x = &storage[0];
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = A(i, j);
  }
  matrix(const matrix &A) : matrix(static_cast<const_ref>(A)) {}
  matrix(matrix &&m) : ref(nullptr, m.M(), m.N()), storage(std::move(m.storage)) {}

  // Construct a diagonal matrix of the value a.
  matrix(const T &a, int M, int N) : ref(nullptr, M, N), storage(M*N) {
    const_ref::x = &storage[0];
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        at(i, j) = (i == j) ? a : 0;
  }

  matrix(int M, int N) : matrix(0, M, N) {}

  matrix &operator = (const_ref A) {
    storage.clear();
    storage.resize(A.M()*A.N());
    const_ref::m = A.M();
    const_ref::n = A.N();
    const_ref::x = &storage[0];
    for (int i = 0; i < M(); i++)
      for (int j = 0; j < N(); j++)
        at(i, j) = A(i, j);
    return *this;
  }
  matrix &operator = (const matrix &A) { return *this = static_cast<const_ref>(A); }
  matrix &operator = (matrix &&A) {
    storage = std::move(A.storage);
    ref::m = A.M();
    ref::n = A.N();
    return *this;
  }
};

/** Basic arithmetic operations for matrix types. */
///@{
template <typename T, int M, int N>
matrix<T, M, N> operator +(const_matrix_ref<T, M, N> A, const_matrix_ref<T, M, N> B) {
  assert(A.M() == B.M() && A.N() == B.N());
  matrix<T, M, N> C(A.M(), A.N());
  for (int i = 0; i < C.M(); i++)
    for (int j = 0; j < C.N(); j++)
      C(i, j) = A(i, j) + B(i, j);
  return C;
}

template <typename T, int M, int N>
matrix<T, M, N> operator -(const_matrix_ref<T, M, N> A, const_matrix_ref<T, M, N> B) {
  assert(A.M() == B.M() && A.N() == B.N());
  matrix<T, M, N> C(A.M(), A.N());
  for (int i = 0; i < C.M(); i++)
    for (int j = 0; j < C.N(); j++)
      C(i, j) = A(i, j) - B(i, j);
  return C;
}

template <typename T, int M, int N, int K>
matrix<T, M, K> operator *(const_matrix_ref<T, M, N> A, const_matrix_ref<T, N, K> B) {
  matrix<T, M, K> C(A.M(), B.N());
  for (int i = 0; i < C.M(); i++) {
    for (int j = 0; j < C.N(); j++) {
      T ij = 0;
      for (int k = 0; k < A.N(); k++)
        ij += A(i, k)*B(k, j);
      C(i, j) = ij;
    }
  }
  return C;
}

template <typename T, int M, int N>
matrix<T, M, N> operator *(T a, const_matrix_ref<T, M, N> B) {
  matrix<T, M, N> C(B.M(), B.N());
  for (int i = 0; i < C.M(); i++)
    for (int j = 0; j < C.N(); j++)
      C(i, j) = a*B(i, j);
  return C;
}

template <typename T, int M, int N>
matrix<T, M, N> operator *(const_matrix_ref<T, M, N> A, T b) {
  matrix<T, M, N> C(A.M(), A.N());
  for (int i = 0; i < C.M(); i++)
    for (int j = 0; j < C.N(); j++)
      C(i, j) = A(i, j)*b;
  return C;
}
///@}

/** Compute the inner product of two vectors. */
///@{
template <typename T, int N>
T dot(const_matrix_ref<T, N, 1> A, const_matrix_ref<T, N, 1> B) {
  // This is a cheap transpose operation.
  const_matrix_ref<T, 1, N> AT(A.elements(), 1, A.M());
  return (AT*B)(0, 0);
}

template <typename T>
T dot(const_matrix_ref<T> A, const_matrix_ref<T> B) {
  // This is a cheap transpose operation.
  const_matrix_ref<T> AT(A.elements(), 1, A.M());
  return (AT*B)(0, 0);
}
///@}

/** Row reduce the augmented matrix \f$[A | B]\f$ in place. */
template <typename T, int M, int N, int implicit_zero, typename BT>
void row_reduce(matrix_ref<T, M, N> A, BT B) {
  assert(A.M() == B.M());
  for (int i = 0; i < A.M(); ++i) {
    // Find a pivot row for this variable.
    int pi = i;
    T max = abs(A(i, i));
    for (int i2 = i + 1; i2 < A.M(); ++i2) {
      T absi2j = abs(A(i2, i));
      if (absi2j > max) {
        pi = i2;
        max = absi2j;
      }
    }

    // Swap pivot row with the current row.
    if (pi != i) {
      for (int j = i; j < A.N(); ++j)
        std::swap(A(i, j), A(pi, j));
      for (int j = 0; j < B.N(); ++j)
        std::swap(B(i, j), B(pi, j));
    }

    // Eliminate the rows after the pivot.
    T p = A(i, i);
    for (int i2 = i + 1; i2 < A.M(); ++i2) {
      T s = A(i2, i)/p;
      for (int j = i + implicit_zero; j < A.N(); ++j)
        A(i2, j) -= A(i, j)*s;
      for (int j = 0; j < B.N(); j++)
        B(i2, j) -= B(i, j)*s;
    }
  }
}

/** An empty matrix to augment with. */
template <typename T>
class null_matrix {
public:
  int M() const { return 0; }
  int N() const { return 0; }

  T& operator() (int i, int j) const { throw std::runtime_error("dereferencing a null matrix."); }
};

/** Row reduce the matrix \f$A\f$ in-place. */
template <typename T, int M, int N>
matrix_ref<T, M, N> row_reduce(matrix_ref<T, M, N> A) {
  return row_reduce<T, M, N, 0>(A, null_matrix<T>());
}

/** Solve \f$A x = [b1 ... bN]\f$. Note that this function mutates the input arguments. */
template <typename T, int N, int Nb>
matrix_ref<T, N, Nb> solve(matrix_ref<T, N, N> A, matrix_ref<T, N, Nb> b) {
  assert(A.M() == A.N() && A.M() == b.M());
  // Row reduce A | b.
  row_reduce<T, N, N, 0>(A, b);

  // A is now upper triangular, so we can solve it.
  for (int i = A.M() - 1; i >= 0; i--) {
    for (int jb = 0; jb < b.N(); jb++) {
      T r = b(i, jb);
      for (int j = i + 1; j < A.N(); j++)
        r -= A(i, j)*b(jb, j);
      b(i, jb) = r/A(i, i);
    }
  }

  return b;
}

/** Check if any matrix element is NaN. */
template <typename T, int M, int N>
bool isnan(const_matrix_ref<T, M, N> A) {
  for (int i = 0; i < A.M(); i++)
    for (int j = 0; j < A.N(); j++)
      if (isnan(A(i, j)))
        return true;
  return false;
}

/** Check that all matrix elements are finite (not infinity or NaN). */
template <typename T, int M, int N>
bool isfinite(const_matrix_ref<T, M, N> A) {
  for (int i = 0; i < A.M(); i++)
    for (int j = 0; j < A.N(); j++)
      if (!isfinite(A(i, j)))
        return false;
  return true;
}

template <typename T, int M, int N>
std::ostream &operator << (std::ostream &os, const_matrix_ref<T, M, N> A) {
  os << '[';
  for (int i = 0; i < A.M(); i++) {
    os << '[';
    for (int j = 0; j < A.N(); j++) {
      os << A(i, j);
      if (j + 1 < A.N()) os << ' ';
    }
    os << ']';
  }
  os << ']';
  return os;
}

template <typename T, int M, int N>
std::istream &operator >> (std::istream &is, matrix_ref<T, M, N> A) {
  is.ignore(std::numeric_limits<std::streamsize>::max(), '[');
  for (int i = 0; i < A.M(); i++) {
    is.ignore(std::numeric_limits<std::streamsize>::max(), '[');
    for (int j = 0; j < A.N(); j++)
      is >> A.at(i, j);
    is.ignore(std::numeric_limits<std::streamsize>::max(), ']');
  }
  is.ignore(std::numeric_limits<std::streamsize>::max(), ']');
  return is;
}

}  // namespace ev3cv

#endif
