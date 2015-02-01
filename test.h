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
