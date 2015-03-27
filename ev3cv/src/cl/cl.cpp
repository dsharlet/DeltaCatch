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

#include "../../include/cl/cl.h"

#include <algorithm>
#include <map>

using namespace std;

namespace ev3cv {
namespace cl {

list<base_arg *> &args() {
  static list<base_arg *> the_args;
  return the_args;
}

base_arg *next_positional_arg() {
  list<base_arg *>::iterator i = find_if(args().begin(), args().end(), [=] (base_arg *i) {
    return i->has_flag(positional) && (i->has_flag(multi) || !i->parsed());
  });
  return i != args().end() ? *i : nullptr;
}

base_arg *find_arg(const char *name) {
  list<base_arg *>::iterator i = find_if(args().begin(), args().end(), [=] (base_arg *i) {
    return !i->has_flag(positional) && i->name() == name;
  });
  return i != args().end() ? *i : nullptr;
}

base_arg *find_arg(char flag) {
  list<base_arg *>::iterator i = find_if(args().begin(), args().end(), [=] (base_arg *i) {
    return !i->has_flag(positional) && i->flag() == flag;
  });
  return i != args().end() ? *i : nullptr;
}

boolean help(
  flag('?'),
  name("help"),
  desc("Show command line options"),
  flags(hidden));

void base_arg::register_arg(base_arg *a) {
  if (!a->name().empty() && find_arg(a->name().c_str()))
    throw runtime_error("redefinition of cl::arg '" + a->name() + "'");
  if (a->flag() != 0 && find_arg(a->flag()))
    throw runtime_error("redefinition of cl::arg '" + string(1, a->flag()) + "'");
  args().push_back(a);
}

void base_arg::unregister_arg(base_arg *a) {
  args().remove(a);
}

void usage(const char *arg0, ostream &os) {
  os << "Usage: " << endl;
  os << "  " << arg0;
  for (auto i : args()) {
    if (i->has_flag(positional)) {
      os << ' ' << i->name();
    }
  }
  os << endl << endl;

  os << "Optional arguments:" << endl;
  map<string, vector<base_arg *>> groups;
  for (auto i : args()) {
    if (!i->has_flag(hidden) && !i->has_flag(positional))
      groups[i->group()].push_back(i);
  }

  for (const auto &i : groups) {
    if (!i.first.empty())
      os << i.first << ":" << endl;
    for (auto j : i.second) {
      os << "  ";
      j->print(os);
    }
    os << endl;
  }
}

void usage(const char *arg0) {
  usage(arg0, cout);
}

void parse(list<const char *> argv) {
  while (!argv.empty()) {
    const char *argi = argv.front();

    base_arg *ai = NULL;
    if (*argi++ == '-') {
      // This is a named argument?
      if (*argi == '-')
        ai = find_arg(argi + 1);
      else
        ai = find_arg(*argi);

      // If we found the argument by name, remove it from the list.
      if (ai)
        argv.pop_front();
      else if (*argi != '-')
        ai = next_positional_arg();
    } else {
      // This is a positional argument.
      ai = next_positional_arg();
    }
    if (ai != NULL) {
      ai->parse(argv);
      ai->set_parsed();
    } else {
      cerr << "Warning: Unused command line option '" << argv.front() << "'." << endl;
      argv.pop_front();
    }
  }
}

void parse(int argc, const char **argv) {
  list<const char *> args;
  for (int i = 0; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  parse(args);
}

void parse(const char *arg0, int argc, const char **argv) {
  parse(argc, argv);

  // If the help flag was set, show usage and exit.
  if (help) {
    usage(arg0);
    exit(1);
  }
}

void parse(const string &arg) {
  list<const char *> args = { arg.c_str() };
  parse(args);
}

void parse(istream &is, char delim) {
  while (is) {
    string line;
    getline(is, line, delim);

    string::size_type space = line.find(' ');
    if (space != string::npos) {
      line[space] = '\0';
      const char *args[] = {
        &line[0],
        &line[space + 1]
      };
      parse(2, args);
    }
  }
}

}  // namespace cl
}  // namespace ev3cv

