#include <ev3cv.h>

#include <fstream>

namespace ev3cv {

std::ostream &null_ostream() {
  static std::ofstream os;
  return os;
}

}  // namespace ev3cv