#ifndef EV3CV_CL_ARG_PORT_H
#define EV3CV_CL_ARG_PORT_H

#include <stdexcept>

#include "cl.h"

namespace ev3cv {
namespace cl {

// Delta robot geometry.
// Comand line argument for EV3 ports.
class arg_port : public cl::arg<std::string> {
public:
  arg_port(
      std::string value,
      const cl::arg_setter &a1 = cl::null_arg_setter(), 
      const cl::arg_setter &a2 = cl::null_arg_setter(), 
      const cl::arg_setter &a3 = cl::null_arg_setter(), 
      const cl::arg_setter &a4 = cl::null_arg_setter(), 
      const cl::arg_setter &a5 = cl::null_arg_setter()) 
    : cl::arg<std::string>(value, a1, a2, a3, a4, a5) {}

  void parse(std::list<const char *> &argv) {
    const char *front = NULL;
    if (!argv.empty()) {
      front = argv.front();
      argv.pop_front();
    }
    
    if (!front || front[0] == 0 || front[1] != 0)
      throw std::runtime_error("unrecognized port");

    switch(std::tolower(front[0])) {
    case 'a': value = "outA"; return;
    case 'b': value = "outB"; return;
    case 'c': value = "outC"; return;
    case 'd': value = "outD"; return;
    case '1': value = "in1"; return;
    case '2': value = "in2"; return;
    case '3': value = "in3"; return;
    case '4': value = "in4"; return;
    }
    throw std::runtime_error("unrecognized port");
  }
};

}  // namespace cl
}  // namespace ev3cv

#endif