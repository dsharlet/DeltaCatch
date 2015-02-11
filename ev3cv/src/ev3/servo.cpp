#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <signal.h>

#include <ev3/servo.h>

namespace ev3cv {

namespace {
std::vector<servo *> servos;
std::mutex servos_lock;

std::thread controller_thread;
}

void controller_main() {
  // Clock to use for controllers.
  typedef std::chrono::high_resolution_clock clock;

  // Sampling period for the controllers.
  static const std::chrono::milliseconds dt(10);

  auto t0 = clock::now();
  for(auto t = t0; ; t += dt) {
    if (dt.count() > 0) {
      std::lock_guard<std::mutex> lock(servos_lock);
      for (auto i : servos) {
        i->tick(dt.count());
      }
      if (servos.empty())
        break;
    }
    std::this_thread::sleep_until(t);
  }
}

servo::servo(const ev3dev::port_type &port) : m_(port), pid_(5000, 5000, 200, 0, 5000) {
  reset();
  
  {
    std::lock_guard<std::mutex> lock(servos_lock);
    servos.push_back(this);
  }

  // If we don't have a controller thread, make one.
  if (!controller_thread.joinable()) {
    std::thread new_thread(controller_main);
    std::swap(controller_thread, new_thread);
  }
}

servo::~servo() {
  servos_lock.lock();
  servos.erase(std::find(servos.begin(), servos.end(), this));

  // If there are no more live motors, wait for the controller thread to complete.
  if (servos.empty()) {
    servos_lock.unlock();
    controller_thread.join();
  } else {
    servos_lock.unlock();
  }

  m_.reset();
}

void servo::run() {
  {
    std::lock_guard<std::mutex> lock(this->lock_);
    pid_.reset();
  }
  m_.set_run_mode(ev3dev::motor::run_mode_forever);
  tick(0);
  m_.run();
}

void servo::stop(bool hold) {
  std::lock_guard<std::mutex> lock(this->lock_);
  m_.set_stop_mode(hold ? ev3dev::motor::stop_mode_hold : ev3dev::motor::stop_mode_coast);
  m_.stop();
  pid_.reset();
}

void servo::reset(int position) {
  std::lock_guard<std::mutex> lock(this->lock_);
  m_.reset();
  m_.set_position(position);
  pid_.reset();
  max_duty_cycle_ = 100;
}

void servo::tick(int dt) {
  std::lock_guard<std::mutex> lock(this->lock_);
  int x = position();
  if (sp_fn_) {
    t_ += dt;
    pid_.set_setpoint(sp_fn_(x, t_, dt));
  }
  int y = pid_.tick(x, dt)/1024;
  m_.set_duty_cycle_setpoint(clamp(y, -max_duty_cycle_, max_duty_cycle_));
}

int servo::position_setpoint() const { 
  return pid_.setpoint(); 
}

void servo::set_position_setpoint(int sp) { 
  std::lock_guard<std::mutex> lock(this->lock_);
  sp_fn_ = nullptr; 
  pid_.set_setpoint(sp); 
}

void servo::set_position_setpoint(std::function<int(int, int, int)> sp_fn) { 
  std::lock_guard<std::mutex> lock(this->lock_);
  sp_fn_ = sp_fn; 
  t_ = 0; 
  pid_.set_setpoint(sp_fn(position(), t_, 0));
}

void servo::set_max_duty_cycle(int x) {
  std::lock_guard<std::mutex> lock(this->lock_);
  max_duty_cycle_ = x;
}

}  // namespace ev3cv