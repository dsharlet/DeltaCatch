#ifndef EV3CV_CIRCULAR_ARRAY_H
#define EV3CV_CIRCULAR_ARRAY_H

#include <cassert>
#include <array>

namespace ev3cv {

// A circular buffer of observations.
template <typename T, size_t N>
class circular_array {
  std::array<T, N> m;
  size_t begin_, end_;

public:
  circular_array() : begin_(0), end_(0) {}
  circular_array(const std::initializer_list<T> &init) : circular_array() {
    for (typename std::initializer_list<T>::const_iterator i = init.begin(); i != init.end(); i++)
      push_back(*i);
  }

  T &at(size_t i) { assert(begin() <= i && i < end()); return m[i % N]; }
  const T &at(size_t i) const { assert(begin() <= i && i < end()); return m[i % N]; }

  size_t begin() const { return begin_; }
  size_t end() const { return end_; }
  size_t size() const { return end_ - begin_; }
  size_t capacity() const { return N; }
  bool empty() const { return size() == 0; }
  
  void push_back(const T &obs) {
    assert(size() < N);
    at(end_++) = obs;
  }

  void push_back(T &&obs) {
    assert(size() < N);
    at(end_++) = std::move(obs);
  }

  void pop_front() {
    assert(size() > 0);
    begin_ += 1;
  }
  
  T &front() { assert(!empty()); return at(begin()); }
  const T &front() const { assert(!empty()); return at(begin()); }
  T &back() { assert(!empty()); return at(end() - 1); }
  const T &back() const { assert(!empty()); return at(end() - 1); }

  void clear() {
    begin_ = end_ = 0;
  }

  T &operator[] (size_t i) { return at(i); }
  const T &operator[] (size_t i) const { return at(i); }
};

}  // namespace ev3cv

#endif