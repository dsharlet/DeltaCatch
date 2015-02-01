#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include <sstream>
#include <vector>
#include <stdexcept>

#include <ev3cv.h>


template <typename T>
void write_calibration_data(std::ostream &os, const calibration_data<T> &cd) {
  for (const auto &set : cd.sets) {
    os << "set " << set.radius;
    if (set.center_valid)
      os << " " << set.center;
    os << std::endl;
    for (const auto &s : set.samples)
      os << "sample " << s.px0 << " " << s.px1 << std::endl;
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