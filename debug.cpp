#include <fstream>
#include <cl/cl.h>

#include "debug.h"

using namespace std;
using namespace ev3cv;

static cl::arg<int> debug(
  0,
  cl::name("debug"),
  cl::desc("Turn on debugging information."));

int dbg_level() { return debug; }

// Get a stream suitable for debug output.
std::ostream &dbg(int level) {
  static std::ofstream unopened;
  if (debug >= level) return cout;
  else return unopened;
}
