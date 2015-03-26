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

#include <ev3cv.h>
#include "test.h"

using namespace std;
using namespace ev3cv;

template <typename T, int N_>
void test_solve(int N) {
  matrix<double, N_, N_> A(N, N);
  matrix<double, N_, N_ == 0 ? 0 : 1> b(N, 1);

  for (int i = 0; i < N; i++) {
    b(i) = randf();
    for (int j = 0; j < N; j++) {
      A(i, j) = randf();
    }
  }

  matrix<double, N_, N_> A_(A);
  matrix<double, N_, N_ == 0 ? 0 : 1> b_(b);

  matrix_ref<double, N_, N_ == 0 ? 0 : 1> x = solve(A_, b_);

  matrix<double, N_, N_ == 0 ? 0 : 1> Ax = A*x;

  for (int i = 0; i < N; i++) {
    ASSERT_LT(abs(Ax(i) - b(i)), 1e-6);
  }
}

template <typename T, int N>
void test_solve() {
  test_solve<T, N>(N);
}

int main(int argc, const char **argv) {
  for (int i = 0; i < 10; i++) {
    for (int N = 1; N < 25; N++)
      test_solve<double, 0>(N);

    test_solve<double, 2>();
    test_solve<double, 5>();
    test_solve<double, 7>();
    test_solve<double, 10>();
    test_solve<double, 16>();
    test_solve<double, 20>();
    test_solve<double, 25>();
  }
  return 0;
}
