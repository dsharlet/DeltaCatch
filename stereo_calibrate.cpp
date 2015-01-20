#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <chrono>

#include "debug.h"
#include "nxtcam.h"
#include "stereo_config.h"

#include "autodiff.h"
#include "matrix.h"
#include "quaternion.h"

using namespace ev3dev;
using namespace std;

static stereo_config stereo;

static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Sample rate of the cameras."));
static cl::arg<int> sample_count(
  50,
  cl::name("sample-count"),
  cl::desc("How many samples to collect."));
static cl::arg<vector3f> sample_origin(
  vector3f(0.0f, 0.0f, 6.0f),
  cl::name("sample-origin"),
  cl::desc("Center of the sampling sphere, in studs."));
static cl::arg<float> sample_radius(
  98.0f,
  cl::name("sample-radius"),
  cl::desc("Radius of the sampling sphere, in studs."));

static cl::arg<float> new_sample_min(
  5.0f,
  cl::name("new-sample-min"),
  cl::desc("Minimum distance for a new sample to be considered, in pixels."));
static cl::arg<float> new_sample_max(
  20.0f,
  cl::name("new-sample-max"),
  cl::desc("Maximum distance for a new sample to be considered, in pixels."));

static cl::arg<int> max_iterations(
  200,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."));
static cl::arg<float> epsilon(
  1e-6,
  cl::name("epsilon"),
  cl::desc("Number to consider to be zero when solving optimization problems."));

static cl::arg<string> output(
  "stereo.cb",
  cl::name("output-file"),
  cl::flags(cl::positional));


template <typename T, int N>
std::ostream &operator << (std::ostream &os, const diff<T, N> &d) {
  return os << d.f;
}

struct sample {
  vector2f e0, e1;

  sample() {}
  sample(const vector2f &e0, const vector2f &e1) : e0(e0), e1(e1) {}
};

vector<sample> collect_samples();

template <typename T>
void dump_config(ostream &os, const camera<T> &cam0, const camera<T> &cam1, const char *prefix = "--") {
  os << prefix << stereo.cam0.focal_length.name() << " " << cam0.focal_length << endl;
  os << prefix << stereo.cam0.distortion.name() << " " << cam0.distortion << endl;
  os << prefix << stereo.cam1.focal_length.name() << " " << cam1.focal_length << endl;
  os << prefix << stereo.cam1.distortion.name() << " " << cam1.distortion << endl;
}

void calibrate(const std::vector<sample> &samples, cameraf &cam0, cameraf &cam1) {
  const size_t M = samples.size();
  const double epsilon_sq = epsilon*epsilon;

  enum variable {
    // Sensor properties
    e0_f,
    e1_f,

    // Distortion
    e0_dx, e0_dy,
    e1_dx, e1_dy,
    
    //e0_ox, e0_oy,
    //e1_ox, e1_oy,

    // The number of variables.
    N,
  };
  typedef diff<double, N> d;
  
  camera<d> cam0_ = camera_cast<d>(cam0);
  cam0_.focal_length = d(cam0.focal_length, e0_f);
  cam0_.distortion = vector2<d>(d(cam0.distortion.x, e0_dx), d(cam0.distortion.y, e0_dy));
  //cam0_.focal_plane.origin = vector2<d>(d(cam0.focal_plane.origin.x, e0_ox), d(cam0.focal_plane.origin.y, e0_oy));

  camera<d> cam1_ = camera_cast<d>(cam1);
  cam1_.focal_length = d(cam1.focal_length, e1_f);
  cam1_.distortion = vector2<d>(d(cam1.distortion.x, e1_dx), d(cam1.distortion.y, e1_dy));
  //cam1_.focal_plane.origin = vector2<d>(d(cam1.focal_plane.origin.x, e1_ox), d(cam1.focal_plane.origin.y, e1_oy));

  double baseline = abs(cam0.transform.origin - cam1.transform.origin);//.baseline;

  const vector3d sample_origin = vector_cast<double>(*::sample_origin);
  const double sample_radius = scalar_cast<double>(*::sample_radius);

  int it;
  for (it = 1; it <= max_iterations; it++) {
    // Compute J^T*J and b.
    matrix<double, N, N> JTJ;
    matrix<double, N, 1> JTy;
    for (size_t i = 0; i < M; i++) {
      const sample &s_i = samples[i];
      vector2<d> f0 = cam0_.sensor_to_focal_plane(vector_cast<d>(s_i.e0));
      vector2<d> f1 = cam1_.sensor_to_focal_plane(vector_cast<d>(s_i.e1));
      
      // z is determined by the stereo disparity.
      d z = baseline/abs(f0 - f1);
      
      vector3<d> x0 = cam0_.focal_plane_to_projection(f0, z);
      vector3<d> x1 = cam1_.focal_plane_to_projection(f1, z);

      // Error in depth from the calibration sphere and x, for both samples.
      d r_z0 = abs(x0 - sample_origin) - sample_radius;
      d r_z1 = abs(x1 - sample_origin) - sample_radius;

      // The focal plane y values should be equal.
      d r_x = f0.y - f1.y;
      
      for (int i = 0; i < N; i++) {
        double Dr_x_i = D(r_x, i);
        double Dr_z0_i = D(r_z0, i);
        double Dr_z1_i = D(r_z1, i);
        // Add this residual to J^T*y.
        JTy(i) -= Dr_x_i*r_x.f + Dr_z0_i*r_z0.f + Dr_z1_i*r_z1.f;
        // Add this residual to J^T*J
        for (int j = 0; j < N; j++)
          JTJ(i, j) += Dr_x_i*D(r_x, j) + Dr_z0_i*D(r_z0, j) + Dr_z1_i*D(r_z1, j);
      }
    }

    // Solve J^T*J*dB = J^T*y.
    matrix_ref<double, N, 1> dB = solve(JTJ, JTy);
    
    if (!isfinite(dB))
      throw runtime_error("optimization diverged");

    dbg(2) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
    
    cam0_.focal_length += dB(e0_f);
    cam0_.distortion += vector2d(dB(e0_dx), dB(e0_dy));
    //cam0_.focal_plane.origin += vector2d(dB(e0_ox), dB(e0_oy));
    
    cam1_.focal_length += dB(e1_f);
    cam1_.distortion += vector2d(dB(e1_dx), dB(e1_dy));
    //cam1_.focal_plane.origin += vector2d(dB(e1_ox), dB(e1_oy));
    
    if (dot(dB, dB) < epsilon_sq) {
      dbg(1) << "  converged on it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
      break;
    }
  }
  
  cam0.focal_length = scalar_cast<float>(cam0_.focal_length);
  cam0.distortion = vector_cast<float>(cam0_.distortion);

  cam1.focal_length = scalar_cast<float>(cam1_.focal_length);
  cam1.distortion = vector_cast<float>(cam1_.distortion);
}

void test_calibrate();

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);
  
  test_calibrate();

  return 0;

  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();

  // Get a list of samples corresponding to observations of an object lying somewhere on the sampling sphere. 
  vector<sample> samples = collect_samples();
  calibrate(samples, cam0, cam1);
  
  dump_config(cout, cam0, cam1);

  // Dump results to output file too.
  ofstream file(output);
  dump_config(file, cam0, cam1);
  
  return 0;
}

vector<sample> collect_samples() {
  vector<sample> samples;
  
  // Start up cameras.
  nxtcam cam0(stereo.cam0.port);
  nxtcam cam1(stereo.cam1.port); 
  cout << "Cameras:" << endl;
  cout << cam0.device_id() << " " << cam0.version() << " (" << cam0.vendor_id() << ")" << endl;
  cout << cam1.device_id() << " " << cam1.version() << " (" << cam1.vendor_id() << ")" << endl;

  cam0.track_objects();
  cam1.track_objects();
  
  // t will increment in regular intervals of T.
  typedef chrono::steady_clock clock;
  auto t = clock::now();
  chrono::microseconds T(static_cast<int>(1e6f/sample_rate + 0.5f));
  while (static_cast<int>(samples.size()) < sample_count) {
    nxtcam::blob_list blobs0 = cam0.blobs();
    nxtcam::blob_list blobs1 = cam1.blobs();

    if (blobs0.size() == 1 && blobs1.size() == 1) {
      vector2f e0 = blobs0.front().center();
      vector2f e1 = blobs1.front().center();

      if (samples.empty()) {
        samples.emplace_back(e0, e1);
        cout << "Recorded sample " << samples.size() << ": " << e0 << ", " << e1 << endl;
      } else {
        float d = max(abs(samples.back().e0 - e0), abs(samples.back().e1 - e1));
        if (new_sample_min < d && d < new_sample_max) {
          samples.emplace_back(e0, e1);
          cout << "Recorded sample " << samples.size() << ": " << e0 << ", " << e1 << endl;
        }
      }
    }

    t += T;
    this_thread::sleep_until(t);
  }

  cam0.stop_tracking();
  cam1.stop_tracking();
  return samples;
}

void test_calibrate() {
  cameraf cam0, cam1;
  tie(cam0, cam1) = stereo.cameras();
  
  //cam0.focal_length = 4.5f;
  
  cam0.distortion.x = 0.01f;
  cam0.distortion.y = 0.02f;

  cam1.distortion.x = 0.03f;
  cam1.distortion.y = 0.04f;

  for (int i = 0; i < 100; i++) {
    vector2f s = vector2f(randf(), randf())*cam0.resolution;
    vector2f f = cam0.sensor_to_focal_plane(s);
    vector2f s_ = cam0.focal_plane_to_sensor(f);

    if (abs(s - s_) > 1e-2f)
      cout << "sensor_to_focal_plane not invertible! " << s << " -> " << f << " -> " << s_ << endl;
  }

  vector<sample> samples;
  while(static_cast<int>(samples.size()) < sample_count) {
    vector3f x = unit(randv3f())*sample_radius + *sample_origin;
    vector2f s0 = cam0.project_to_sensor(x);
    vector2f s1 = cam1.project_to_sensor(x);
    if (0 <= s0.x && s0.x < cam0.resolution.x &&
        0 <= s0.y && s0.y < cam0.resolution.y &&
        0 <= s1.x && s1.x < cam1.resolution.x &&
        0 <= s1.y && s1.y < cam1.resolution.y) {
      samples.emplace_back(s0, s1);
    }
  }

  dump_config(dbg(1), cam0, cam1);

  tie(cam0, cam1) = stereo.cameras();
  calibrate(samples, cam0, cam1);
  dump_config(dbg(1), cam0, cam1);
}

