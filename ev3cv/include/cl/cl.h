#ifndef EV3CV_CL_CL_H
#define EV3CV_CL_CL_H

#include <string>
#include <iostream>
#include <sstream>
#include <list>
#include <vector>
#include <stdexcept>

namespace ev3cv {
namespace cl {

// Flags to control the behavior of an arg.
enum arg_flag {
  positional = 1 << 0,
  multi = 1 << 1,

  hidden = 1 << 31,
};

class base_arg;

// Show application usage.
void usage(const char *arg0);
// Parse some command line arguments.
void parse(const char *arg0, int argc, const char **argv);
void parse(int argc, const char **argv);
void parse(const std::string &arg);

class arg_setter {
public:
  arg_setter() {}
  arg_setter(const arg_setter &) = delete;
  arg_setter(arg_setter &&) = delete;
  arg_setter &operator = (const arg_setter &) = delete;
  arg_setter &operator = (arg_setter &&) = delete;

  virtual ~arg_setter() {}

  virtual void apply(base_arg *arg) const = 0;
};

class null_arg_setter : public arg_setter {
public:
  void apply(base_arg *) const {}
};

// Base argument only registers and unregisters itself.
class base_arg {
protected:
  char flag_;
  std::string name_;
  std::string desc_;
  std::string group_;
  unsigned flags_;
  bool parsed_;
    
  static void register_arg(base_arg *a);
  static void unregister_arg(base_arg *a);

  base_arg(const arg_setter &a1, 
            const arg_setter &a2,  
            const arg_setter &a3, 
            const arg_setter &a4, 
            const arg_setter &a5, 
            const arg_setter &a6) : flag_(0), flags_(0), parsed_(false) {
    a1.apply(this);
    a2.apply(this);
    a3.apply(this);
    a4.apply(this);
    a5.apply(this);
    a6.apply(this);
    register_arg(this);
  }

  virtual ~base_arg() { unregister_arg(this); }

public:
  virtual void parse(std::list<const char *> &argv) = 0;
  virtual void print(std::ostream &os) { os << "   " << desc_ << std::endl; }
    
  char flag() const { return flag_; }
  const std::string &name() const { return name_; }
  const std::string &desc() const { return desc_; }
  const std::string &group() const { return group_; }
  bool has_flag(arg_flag f) const { return (flags_ & f) != 0; }

  void set_flag(char f) { flag_ = f; }
  void set_name(std::string name) { name_ = std::move(name); }
  void set_desc(std::string desc) { desc_ = std::move(desc); }
  void set_group(std::string group) { group_ = std::move(group); }
  void add_flags(unsigned flags) { flags_ |= flags; }
  void remove_flags(unsigned flags) { flags_ &= (~flags); }

  bool parsed() const { return parsed_; }
  void set_parsed() { parsed_ = true; }
};
  
class name : public arg_setter {
  std::string v;
public:
  name(std::string v) : v(std::move(v)) {}
  void apply(base_arg *a) const { a->set_name(v); }
};

class flag : public arg_setter {
  char v;
public:
  flag(char v) : v(v) {}
  void apply(base_arg *a) const { a->set_flag(v); }
};

class desc : public arg_setter {
  std::string v;
public:
  desc(std::string v) : v(std::move(v)) {}
  void apply(base_arg *a) const { a->set_desc(v); }
};
  
class flags : public arg_setter {
  unsigned v;
public:
  flags(unsigned v) :  v(v) {}
  void apply(base_arg *a) const { a->add_flags(v); }
};

class remove_flags : public arg_setter {
  unsigned v;
public:
  remove_flags(unsigned v) :  v(v) {}
  void apply(base_arg *a) const { a->remove_flags(v); }
};

class group : public arg_setter {
  std::string v;
public:
  group(std::string v) : v(std::move(v)) {}
  void apply(base_arg *a) const { a->set_group(v); }
};

// A basic argument.
template <typename T>
class arg : public base_arg {
protected:
  T value;

public:
  operator const T&() const { return value; }
  const T* operator -> () const { return &value; }
  const T& operator * () const { return value; }

  operator T&() { return value; }
  T* operator -> () { return &value; }
  T& operator * () { return value; }

  void operator = (const T& v) { value = v; }
    
  arg(T value, 
      const arg_setter &a1 = null_arg_setter(), 
      const arg_setter &a2 = null_arg_setter(), 
      const arg_setter &a3 = null_arg_setter(), 
      const arg_setter &a4 = null_arg_setter(), 
      const arg_setter &a5 = null_arg_setter(), 
      const arg_setter &a6 = null_arg_setter()) 
    : base_arg(a1, a2, a3, a4, a5, a6), value(value) {}

  void parse(std::list<const char *> &argv) {
    std::stringstream ss;
    if (!argv.empty()) {
      ss.str(argv.front());
      argv.pop_front();
    }
    ss >> value;
    if (ss.fail())
      throw std::runtime_error("failed to parse value for argument '" + name() + "'");
  }

  void print(std::ostream &os) {
    if (flag_ != 0) os << "-" << flag() << ", ";
    os << "--" << name();
    if (!desc().empty()) os << ": " << desc();
    os << " (" << value << ")" << std::endl;
  }
};
  
// A flag indicating its presence or not.
class boolean : public arg<bool> {
public:
  using arg::operator =;
    
  boolean(
      const arg_setter &a1 = null_arg_setter(), 
      const arg_setter &a2 = null_arg_setter(), 
      const arg_setter &a3 = null_arg_setter(), 
      const arg_setter &a4 = null_arg_setter(), 
      const arg_setter &a5 = null_arg_setter(), 
      const arg_setter &a6 = null_arg_setter()) 
    : arg<bool>(false, a1, a2, a3, a4, a5, a6) {}

  void parse(std::list<const char *> &argv) {
    value = true; 
  }

  void print(std::ostream &os) {
    if (flag_ != 0) os << "-" << flag() << ", ";
    os << "--" << name();
    if (!desc().empty()) os << ": " << desc();
    os << std::endl;
  }
};

// A positional argument that accepts many values.
template <typename T>
class base_arg_list : public base_arg {
protected:
  std::vector<T> values_;

public:
  typedef typename std::vector<T>::iterator iterator;
  typedef typename std::vector<T>::const_iterator const_iterator;

  iterator begin() { return values_.begin(); }
  const_iterator begin() const { return values_.begin(); }
  iterator end() { return values_.end(); }
  const_iterator end() const { return values_.end(); }
  size_t size() const { return values_.size(); }
  T& operator[] (size_t i) { return values_[i]; }
  const T &operator[] (size_t i) const { return values_[i]; }
  bool empty() const { return values_.empty(); }
    
  base_arg_list(
      const arg_setter &a1 = null_arg_setter(), 
      const arg_setter &a2 = null_arg_setter(), 
      const arg_setter &a3 = null_arg_setter(), 
      const arg_setter &a4 = null_arg_setter(), 
      const arg_setter &a5 = null_arg_setter()) 
    : base_arg(cl::flags(multi), a1, a2, a3, a4, a5) {}

  void print(std::ostream &os) {
    if (flag_ != 0) os << "-" << flag() << ", ";
    os << "--" << name();
    if (!desc().empty()) os << ": " << desc();
    os << " (list)" << std::endl;
  }
};

// A positional argument that accepts many values.
template <typename T>
class arg_list : public base_arg_list<T> {
protected:
  using base_arg_list<T>::values_;

public:
  using base_arg_list<T>::name;
  using base_arg_list<T>::begin;
  using base_arg_list<T>::end;
  using base_arg_list<T>::size;
  using base_arg_list<T>::empty;
  using base_arg_list<T>::operator [];
  using base_arg_list<T>::print;
    
  arg_list(
      const arg_setter &a1 = null_arg_setter(), 
      const arg_setter &a2 = null_arg_setter(), 
      const arg_setter &a3 = null_arg_setter(), 
      const arg_setter &a4 = null_arg_setter(), 
      const arg_setter &a5 = null_arg_setter()) 
    : base_arg_list<T>(a1, a2, a3, a4, a5) {}

  void parse(std::list<const char *> &argv) {
    std::stringstream ss(argv.front());
    argv.pop_front();
    T v;
    ss >> v;
    if (ss.fail())
      throw std::runtime_error("failed to parse value for argument '" + name() + "'");
      
    values_.push_back(v);
  }
};

}  // namespace cl
}  // namespace ev3cv

#endif