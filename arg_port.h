#ifndef PORT_ARG_H
#define PORT_ARG_H

#include <stdexcept>

#include <ev3dev.h>
#include <cl.h>

namespace ev3 = ev3dev;

// Delta robot geometry.
// Comand line argument for EV3 ports.
class arg_port : public cl::arg<ev3::port_type> {
public:
  arg_port(
      ev3::port_type value,
      const cl::arg_setter &a1 = cl::null_arg_setter(), 
      const cl::arg_setter &a2 = cl::null_arg_setter(), 
      const cl::arg_setter &a3 = cl::null_arg_setter(), 
      const cl::arg_setter &a4 = cl::null_arg_setter(), 
      const cl::arg_setter &a5 = cl::null_arg_setter()) 
    : cl::arg<ev3::port_type>(value, a1, a2, a3, a4, a5) {}

  void parse(std::list<const char *> &argv) {
    const char *front = NULL;
    if (!argv.empty()) {
      front = argv.front();
      argv.pop_front();
    }
    
    if (!front || front[0] == 0 || front[1] != 0)
      throw std::runtime_error("unrecognized port");

    switch(std::tolower(front[0])) {
    case 'a': *this = ev3::OUTPUT_A; return;
    case 'b': *this = ev3::OUTPUT_B; return;
    case 'c': *this = ev3::OUTPUT_C; return;
    case 'd': *this = ev3::OUTPUT_D; return;
    case '1': *this = ev3::INPUT_1; return;
    case '2': *this = ev3::INPUT_2; return;
    case '3': *this = ev3::INPUT_3; return;
    case '4': *this = ev3::INPUT_4; return;
    }
    throw std::runtime_error("unrecognized port");
  }
};

#endif