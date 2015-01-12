#include <fstream>
#include <cl.h>

#include "debug.h"

using namespace std;

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
