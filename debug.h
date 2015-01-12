#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>

int dbg_level();

// Get a stream suitable for debug output.
std::ostream &dbg(int level);

#endif
