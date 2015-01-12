#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>

#include "nxtcam.h"
#include "arg_port.h"

using namespace ev3dev;
using namespace std;

static arg_port port(
  INPUT_4,
  cl::name("port"),
  cl::desc("Port the NXTcam is connected to."));

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);
  
  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);

  nxtcam cam(port);

  cout << "Connected to camera: " << cam.device_id() << ", " << cam.version() << ", " << cam.vendor_id() << std::endl;

  cam.track_objects();
  this_thread::sleep_for(chrono::milliseconds(500));

  while(true) {
    nxtcam::blob_list blobs = cam.blobs();

    cout << '\r';
    for (nxtcam::blob_list::iterator i = blobs.begin(); i != blobs.end(); i++) {
      cout << setw(2) << i->color << setw(4) << (i->x2 + i->x1) / 2 << setw(4) << (i->y1 + i->y2) / 2;
    }
    for (size_t i = blobs.size(); i < 8; i++) {
      cout << "          ";
    }
    cout.flush();

    this_thread::sleep_for(chrono::milliseconds(33));
  }

  return 0;
}