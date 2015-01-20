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
  
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();
  const float baseline = stereo.baseline;

  while (true) {
    nxtcam::blob_list blobs0 = nxtcam0.blobs();
    nxtcam::blob_list blobs1 = nxtcam1.blobs();

    vector2f f0, f1;
    if (!blobs0.empty()) {
      const nxtcam::blob &i = blobs0.front();
      f0 = cam0.sensor_to_focal_plane(i.center());
    }
    if (!blobs1.empty()) {
      const nxtcam::blob &i = blobs1.front();
      f1 = cam1.sensor_to_focal_plane(i.center());
    }

    string eraser;

    if (!blobs0.empty() && !blobs1.empty()) {
      // z is determined by the stereo disparity.
      float z = baseline/(f1.x - f0.x);
      
      vector3f x0 = cam0.focal_plane_to_projection(f0, z);
      vector3f x1 = cam1.focal_plane_to_projection(f1, z);

      stringstream ss;
      ss << fixed << showpoint << setprecision(3);
      ss << "(" << f0 << ", " << f1 << ") -> (" << x0 << ",  " << x1 << ")";
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