// Copyright 2015 Google, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
