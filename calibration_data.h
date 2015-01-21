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
    vector2<T> e0, e1;

    sample() {}
    sample(const vector2<T> &e0, const vector2<T> &e1) : e0(e0), e1(e1) {}
  };

  struct set {
    bool center_valid;
    vector3<T> center;
    T radius;
    std::vector<sample> samples;
  };
  std::vector<set> sets;
};

template <typename T>
void write_calibration_data(std::ostream &os, const calibration_data<T> &cd) {
  for (const auto &set : cd.sets) {
    os << "set " << set.samples.size() << " " << set.radius;
    if (set.center_valid)
      os << " " << set.center;
    os << endl;
    for (const auto &s : set.samples)
      os << "sample " << s.e0 << " " << s.e1 << endl;
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
        set.center_valid = line.good();
        cd.sets.push_back(std::move(set));
      }
    } else if (cmd == "sample") {
      if (cd.sets.empty())
        throw std::runtime_error("calibration data missing set descriptor");
      typename calibration_data<T>::sample s;
      line >> s.e0 >> s.e1;
      if (line.good())
        cd.sets.back().samples.push_back(s);
    }
  }
  return cd;
}

#endif