#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <chrono>

#include "nxtcam.h"
#include "stereo_config.h"

using namespace ev3dev;
using namespace std;

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);
  
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();
  
  float baseline = abs(cam1.x - cam0.x);
  if (baseline < 1e-6f)
    throw runtime_error("camera baseline is zero");
  vector3f b = unit(cam1.x - cam0.x);

  nxtcam nxtcam0(stereo.cam0.port);
  nxtcam nxtcam1(stereo.cam1.port); 
  cout << "Cameras:" << endl;
  cout << nxtcam0.device_id() << " " << nxtcam0.version() << " (" << nxtcam0.vendor_id() << ")" << endl;
  cout << nxtcam1.device_id() << " " << nxtcam1.version() << " (" << nxtcam1.vendor_id() << ")" << endl;

  nxtcam0.track_objects();
  nxtcam1.track_objects();

  // t will increment in regular intervals of T.
  typedef chrono::high_resolution_clock clock;
  auto t = clock::now();
  chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
  
  string eraser;
  while (true) {
    nxtcam::blob_list blobs0 = nxtcam0.blobs();
    nxtcam::blob_list blobs1 = nxtcam1.blobs();

    if (!blobs0.empty() && !blobs1.empty()) {
      const nxtcam::blob &b0 = blobs0.front();
      const nxtcam::blob &b1 = blobs1.front();
      
      vector3f x0 = cam0.sensor_to_projection(b0.center(), 1.0f) - cam0.x;
      vector3f x1 = cam1.sensor_to_projection(b1.center(), 1.0f) - cam1.x;

      // z is determined by the stereo disparity.
      float z = baseline/(dot(x0, b) - dot(x1, b));

      // Move the points from the focal plane to the (parallel) plane containing z and add the camera origins.
      x0 = x0*z + cam0.x;
      x1 = x1*z + cam1.x;
      
      stringstream ss;
      ss << fixed << showpoint << setprecision(3);
      ss << "x0=" << x0 << ",  x1=" << x1 << ", ||x0 - x1||=" << abs(x0 - x1);
      string msg = ss.str();
      if (msg.length() > eraser.length())
        eraser = string(msg.length(), ' ');

      cout << msg << string(eraser.size() - msg.size(), ' ');
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