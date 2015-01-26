#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <chrono>

#include "nxtcam.h"
#include "stereo_config.h"
#include "delta_robot_args.h"

using namespace ev3dev;
using namespace std;

static cl::group motor_control("Motor control");
static cl::arg<mode_type> regulation_mode(
  "on", 
  cl::name("regulation-mode"), 
  cl::desc("One of: 'on', 'off'."),
  motor_control);
static cl::arg<int> pulses_per_second(
  700, 
  cl::name("pulses-per-second"), 
  cl::desc("Pulses/second for when --regulation-on is specified."),
  motor_control);
static cl::arg<int> duty_cycle(
  100,
  cl::name("duty-cycle"), 
  cl::desc("Duty cycle for when --regulation-on is not specified."),
  motor_control);
static cl::arg<int> ramp(
  0, 
  cl::name("ramp"), 
  cl::desc("Ramp time, in ms."),
  motor_control);

static delta_robot_args delta_geometry("", "Delta robot geometry");

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

static cl::arg<float> scale(
  1.0f,
  cl::name("scale"),
  cl::desc("Ratio of robot movement to object movement."));

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);
  
  nxtcam nxtcam0(stereo.cam0.port);
  nxtcam nxtcam1(stereo.cam1.port); 
  cout << "Cameras:" << endl;
  cout << nxtcam0.device_id() << " " << nxtcam0.version() << " (" << nxtcam0.vendor_id() << ")" << endl;
  cout << nxtcam1.device_id() << " " << nxtcam1.version() << " (" << nxtcam1.vendor_id() << ")" << endl;

  thread nxtcam_init_thread([&] () {
    nxtcam0.track_objects();
    nxtcam1.track_objects();
    cout << "Tracking objects..." << endl;
  });

  // Initialize the delta robot.
  delta_robot delta(delta_geometry.geometry());
  if (scale > 0.0f) {
    delta.init();

    // Bask in the glory of the calibration result for a moment.
    this_thread::sleep_for(chrono::milliseconds(500));

    // Set the motor parameters.
    delta.set_regulation_mode(regulation_mode);
    delta.set_pulses_per_second_setpoint(pulses_per_second);
    delta.set_duty_cycle_setpoint(duty_cycle);
    delta.set_ramp_up(ramp);
    delta.set_ramp_down(ramp);
  }

  nxtcam_init_thread.join();
  
  pair<vector3f, float> volume = delta.get_volume();

  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();
  
  float baseline = abs(cam1.x - cam0.x);
  if (baseline < 1e-6f)
    throw runtime_error("camera baseline is zero");
  vector3f b = unit(cam1.x - cam0.x);

  // t will increment in regular intervals of T.
  typedef chrono::high_resolution_clock clock;
  auto t = clock::now();
  chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
  
  vector3f origin(0.0f, 0.0f, 0.0f);
  
  string eraser;
  while (true) {
    nxtcam::blob_list blobs0 = nxtcam0.blobs();
    nxtcam::blob_list blobs1 = nxtcam1.blobs();

    if (blobs0.size() == 1 && blobs1.size() == 1) {
      const nxtcam::blob &b0 = blobs0.front();
      const nxtcam::blob &b1 = blobs1.front();
      
      vector3f x0 = cam0.sensor_to_projection(b0.center(), 1.0f) - cam0.x;
      vector3f x1 = cam1.sensor_to_projection(b1.center(), 1.0f) - cam1.x;

      // z is determined by the stereo disparity.
      float z = baseline/(dot(x0, b) - dot(x1, b));

      // Move the points from the focal plane to the (parallel) plane containing z and add the camera origins.
      x0 = x0*z + cam0.x;
      x1 = x1*z + cam1.x;
      vector3f x = (x0 + x1)/2;
      
      stringstream ss;
      ss << fixed << showpoint << setprecision(3);
      ss << "x=" << x << ", ||x0 - x1||=" << abs(x0 - x1);
      string msg = ss.str();
      if (msg.length() > eraser.length())
        eraser = string(msg.length(), ' ');
      cout << msg << string(eraser.size() - msg.size(), ' ');
      
      if (dot(origin, origin) == 0.0f)
        origin = x*scale - volume.first;

      x = x*scale - origin;

      if (scale > 0.0f) {
        try {
          // Move to the position.
          delta.run_to(x);
        } catch(runtime_error &) {

        }
      }
    } else {
      cout << eraser;
    }
    cout << "\r";
    cout.flush();

    t += T;
    this_thread::sleep_until(t);
  }

  return 0;
}