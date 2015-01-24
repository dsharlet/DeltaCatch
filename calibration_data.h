#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include <sstream>
#include <vector>
#include <stdexcept>

#include "vector2.h"
#include "vector3.h"

// Calibration data is a list of sets of samples from spheres.
template <typename T>
struct calibration_data {

  struct sample {
    vector2<T> px0, px1;

    sample() {}
    sample(const vector2<T> &px0, const vector2<T> &px1) : px0(px0), px1(px1) {}
  };

  struct set {
    bool center_valid;
    vector3<T> center;
    T radius;
    std::vector<sample> samples;

    set() : center_valid(false) {}
  };
  std::vector<set> sets;

  std::size_t sample_count() const {
    std::size_t samples = 0;
    for (const auto &i : sets) 
      samples += i.samples.size();
    return samples;
  }
};

template <typename T>
void write_calibration_data(std::ostream &os, const calibration_data<T> &cd) {
  for (const auto &set : cd.sets) {
    os << "set " << set.radius;
    if (set.center_valid)
      os << " " << set.center;
    os << endl;
    for (const auto &s : set.samples)
      os << "sample " << s.px0 << " " << s.px1 << endl;
  }
}

template <typename T>
calibration_data<T> read_calibration_data(std::istream &is) {
  calibration_data<T> cd;
  while (is.good()) {
    std::string line_buf;
    getline(is, line_buf);
    std::stringstream line(line_buf);

    std::string cmd;
    line >> cmd;
    if (cmd == "set") {
      typename calibration_data<T>::set set;
      line >> set.radius;
      if (line.good()) {
        line >> set.center;
        set.center_valid = !line.bad();
      }
      cd.sets.push_back(std::move(set));
    } else if (cmd == "sample") {
      if (cd.sets.empty())
        throw std::runtime_error("calibration data missing set descriptor");
      typename calibration_data<T>::sample s;
      line >> s.px0 >> s.px1;
      if (!line.bad())
        cd.sets.back().samples.push_back(s);
    }
  }
  return cd;
}

#endif