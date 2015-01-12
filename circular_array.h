#ifndef CIRCULAR_ARRAY_H
#define CIRCULAR_ARRAY_H

#include <array>

// A circular buffer of observations.
template <typename T, int N>
class circular_array {
  std::array<T, N> m;
  size_t begin_, end_;

public:
  circular_array() : begin_(0), end_(0) {}

  T &at(size_t i) { return m[i % N]; }
  const T &at(size_t i) const { return m[i % N]; }

  size_t begin() const { return begin_; }
  size_t end() const { return end_; }
  size_t size() const { return end_ - begin_; }
  bool empty() const { return size() == 0; }
  
  void push_back(const T &obs) {
    if (size() >= N)
      throw std::runtime_error("circular_array is full.");
    at(end_++) = obs;
  }

  void push_back(T &&obs) {
    if (size() >= N)
      throw std::runtime_error("circular_array is full.");
    at(end_++) = std::move(obs);
  }

  void pop_front() {
    if (size() <= 0)
      throw std::runtime_error("circular_array is empty.");
    begin_ += 1;
  }
  
  T &front() { return at(begin()); }
  const T &front() const { return at(begin()); }
  T &back() { return at(end() - 1); }
  const T &back() const { return at(end() - 1); }

  void clear() {
    begin_ = end_ = 0;
  }

  T &operator[] (size_t i) { return at(i); }
  const T &operator[] (size_t i) const { return at(i); }
};

#endif