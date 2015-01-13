#include <thread>

#include "delta_hand.h"

using namespace std;
using namespace ev3dev;

delta_hand::delta_hand(
    ev3::port_type a, ev3::port_type b, ev3::port_type c, ev3::port_type hand,
    float base, float effector, float bicep, float forearm, int theta_max,
    bool find_limits_now) 
  : delta_robot(a, b, c, base, effector, bicep, forearm, theta_max, false)
  , hand(hand), grab_close(0), grab_open(0) {

  // Don't use the base class because our overridden definition of find_limits won't be called.
  if (find_limits_now)
    find_limits();
} 

void delta_hand::find_limits() {
  // Run the base configuration in a separate thread.
  thread base([=]() { 
    delta_robot::find_limits(); 
  });

  const auto stall_time = chrono::milliseconds(200);
    
  // Start running the grabber motor indefinitely.
  hand.set_run_mode(motor::run_mode_forever);
  hand.set_stop_mode(motor::stop_mode_coast);
  hand.set_regulation_mode(motor::mode_off);
  hand.set_duty_cycle_setpoint(-50);
  grab_close = hand.position();
  hand.run();
  
  // Wait until all the motors hit the zero position.
  while (hand.running()) {
    this_thread::sleep_for(stall_time);
    int pos = hand.position();
    if (pos >= grab_close)
      hand.stop();
    else
      grab_close = pos;
  }
  grab_open = grab_close;

  hand.set_duty_cycle_setpoint(50);
  hand.run();

  while (hand.running()) {
    this_thread::sleep_for(stall_time);
    int pos = hand.position();
    if (pos <= grab_open)
      hand.stop();
    else
      grab_open = pos;
  }
  
  // We always want the hand to move as quickly as possible.
  hand.set_run_mode(motor::run_mode_position);
  hand.set_duty_cycle_setpoint(100);
   
  // Wait for the base's find_limits.
  base.join();
}
