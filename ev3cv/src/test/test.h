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

#include <iostream>
#include <cassert>
#include <cmath>

using namespace std;

template <typename T, typename U>
inline void assert_lt(T l, U r, const char *msg) {
  if (!(l < r)) {
    cerr << "Assertion failed: " << msg << " " << l << " !< " << r << endl;
  }
}

template <typename T, typename U>
inline void assert_eq(T l, U r, const char *msg) {
  if (l != r) {
    cerr << "Assertion failed: " << msg << " " << l << " != " << r << endl;
  }
}

inline void assert_true(bool c, const char *msg) {
  if (!c) {
    cerr << "Assertion failed: " << msg << endl;
  }
}

inline void assert_false(bool c, const char *msg) {
  if (c) {
    cerr << "Assertion failed: " << msg << endl;
  }
}

#define ASSERT_LT(x, y) assert_lt(x, y, #x" < "#y)
#define ASSERT_EQ(x, y) assert_lt(x, y, #x" == "#y)
#define ASSERT_TRUE(x) assert_true(x, #x)
#define ASSERT_FALSE(x) assert_false(x, "!"#x)
