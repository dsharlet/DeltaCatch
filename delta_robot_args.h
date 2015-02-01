#ifndef DELTA_ROBOT_ARGS_H
#define DELTA_ROBOT_ARGS_H

#include <cl/arg_port.h>
#include "delta_robot.h"

// Define the command line parameters to describe a delta_robot.
struct delta_robot_args {
  cl::arg_port arm0{
    ev3::OUTPUT_A,
    cl::name("arm0"),
    cl::desc("The motor port to drive arm 0.")};
  cl::arg_port arm1{
    ev3::OUTPUT_B,
    cl::name("arm1"),
    cl::desc("The motor port to drive arm 1.")};
  cl::arg_port arm2{
    ev3::OUTPUT_C,
    cl::name("arm2"),
    cl::desc("The motor port to drive arm 2.")};
  cl::arg<float> base{
    14.5f,
    cl::name("base-radius"),
    cl::desc("Distance between the center of the base and the shoulder joints, in studs.")};
  cl::arg<float> effector{
    4.5f,
    cl::name("effector-radius"),
    cl::desc("Distance between the center of the effector and the wrist joints, in studs.")};
  cl::arg<float> bicep{
    15.0f,
    cl::name("bicep-length"),
    cl::desc("Distance between the shoulder and elbow joints, in studs.")};
  cl::arg<float> forearm{
    28.0f,
    cl::name("forearm-length"),
    cl::desc("Distance between the elbow and wrist joints, in studs.")};
  cl::arg<int> theta_max{
    100,
    cl::name("theta-max"),
    cl::desc("Theta at the shoulder joint at the reaching position, in degrees.")};

  delta_robot_args(const std::string &prefix = "", const std::string &group = "") {
    if (!prefix.empty())
      add_prefix(prefix);
    if (!group.empty())
      set_group(group);
  }
  
  void add_prefix(const std::string &prefix) {
    arm0.set_name(prefix + "-" + arm0.name());
    arm1.set_name(prefix + "-" + arm1.name());
    arm2.set_name(prefix + "-" + arm2.name());
    base.set_name(prefix + "-" + base.name());
    effector.set_name(prefix + "-" + effector.name());
    bicep.set_name(prefix + "-" + bicep.name());
    forearm.set_name(prefix + "-" + forearm.name());
    theta_max.set_name(prefix + "-" + theta_max.name());
  }

  void set_group(const std::string &group) {
    arm0.set_group(group);
    arm1.set_group(group);
    arm2.set_group(group);
    base.set_group(group);
    effector.set_group(group);
    bicep.set_group(group);
    forearm.set_group(group);
    theta_max.set_group(group);
  }

  delta_robot_geometry geometry() const {
    delta_robot_geometry g;
    g.arm0 = arm0;
    g.arm1 = arm1;
    g.arm2 = arm2;
    g.base = base;
    g.effector = effector;
    g.bicep = bicep;
    g.forearm = forearm;
    g.theta_max = theta_max;
    return g;
  }
};

#endif