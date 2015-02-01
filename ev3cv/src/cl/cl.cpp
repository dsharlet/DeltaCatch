#include "../../include/cl/cl.h"

#include <algorithm>
#include <map>

namespace ev3cv {
namespace cl {

std::list<base_arg *> &args() {
  static std::list<base_arg *> the_args;
  return the_args;
}

base_arg *next_positional_arg() {
  std::list<base_arg *>::iterator i = std::find_if(args().begin(), args().end(), [=] (base_arg *i) {
    return i->has_flag(positional) && (i->has_flag(multi) || !i->parsed());
  });
  return i != args().end() ? *i : nullptr;
}

base_arg *find_arg(const char *name) {
  std::list<base_arg *>::iterator i = std::find_if(args().begin(), args().end(), [=] (base_arg *i) { 
    return !i->has_flag(positional) && i->name() == name; 
  });
  return i != args().end() ? *i : nullptr;
}
  
base_arg *find_arg(char flag) {
  std::list<base_arg *>::iterator i = std::find_if(args().begin(), args().end(), [=] (base_arg *i) { 
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
  args().push_back(a);
}

void base_arg::unregister_arg(base_arg *a) {
  args().remove(a);
} 

void usage(const char *arg0, std::ostream &os) {
  os << "Usage: " << std::endl;
  os << "  " << arg0;
  for (auto i : args()) {
    if (i->has_flag(positional)) {
      os << ' ' << i->name();
    }
  }
  os << std::endl << std::endl;
    
  os << "Optional arguments:" << std::endl;
  std::map<std::string, std::vector<base_arg *>> groups;
  for (auto i : args()) {
    if (!i->has_flag(hidden) && !i->has_flag(positional))
      groups[i->group()].push_back(i);
  }

  for (const auto &i : groups) {
    if (!i.first.empty())
      os << i.first << ":" << std::endl;
    for (auto j : i.second) {
      os << "  ";
      j->print(os);
    }
    os << std::endl;
  }
}

void usage(const char *arg0) {
  usage(arg0, std::cout);
}

void parse(std::list<const char *> argv) {
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
      std::cerr << "Warning: Unused command line option '" << argv.front() << "'." << std::endl;
      argv.pop_front();
    }
  }
}
  
void parse(int argc, const char **argv) {
  std::list<const char *> args;
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
  
void parse(const std::string &arg) {
  std::list<const char *> args = { arg.c_str() };
  parse(args);
}

}  // namespace cl
}  // namespace ev3cv

