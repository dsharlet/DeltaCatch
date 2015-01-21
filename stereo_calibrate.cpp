#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>

#include "cl.h"
#include "debug.h"
#include "camera.h"
#include "autodiff.h"
#include "matrix.h"

#include "calibration_data.h"

using namespace std;

static cl::arg<int> max_iterations(
  50,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."));
static cl::arg<double> epsilon(
  1e-6,
  cl::name("epsilon"),
  cl::desc("Number to consider to be zero when solving optimization problems."));

static cl::arg<string> output(
  "stereo.cb",
  cl::name("output-file"),
  cl::flags(cl::positional));

static cl::boolean test(
  cl::name("test"),
  cl::desc("Generate simulated calibration samples."));

template <typename T, int N>
std::ostream &operator << (std::ostream &os, const diff<T, N> &d) {
  return os << d.f;
}

template <typename T>
void dump_config(ostream &os, const char *prefix, const camera<T> &cam) {
  os << prefix << "distortion " << cam.d << endl;
  os << prefix << "calibration " << cam.K() << endl;
  os << prefix << "orientation " << cam.R << endl;
  os << prefix << "origin " << cam.x << endl;
}

template <typename T>
void dump_config(ostream &os, const camera<T> &cam_0, const camera<T> &cam_1) {
  dump_config(os, "--cam0-", cam_0);
  dump_config(os, "--cam1-", cam_1);
}

template <typename T, int N>
vector2<diff<T, N>> v2d(const vector2<T> &f, int n) {
  vector2<diff<T, N>> v;
  v.x = d(f.x, n + 0);
  v.y = d(f.y, n + 1);
  return v;
}

template <typename T, int N>
vector3<diff<T, N>> v3d(const vector3<T> &f, int n) {
  vector2<diff<T, N>> v;
  v.x = d(f.x, n + 0);
  v.y = d(f.y, n + 1);
  v.z = d(f.y, n + 2);
  return v;
}

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);
  
  // Get a list of samples corresponding to observations of an object lying somewhere on the sampling sphere. 
  const double epsilon_sq = epsilon*epsilon;
  
  typedef diff<double, 100> d;
  calibration_data<d> cd;

  camera<d> cam_0, cam_1;

  int N = 0;
  cam_0.d.x = d(0, N++); cam_0.d.y = d(0, N++);
  cam_0.a.x = d(1, N++); cam_0.a.y = d(1, N++);
  cam_0.s = d(0, N++);
  cam_0.t.x = d(0, N++); cam_0.t.y = d(0, N++);
  cam_0.R.a = d(1, N++); cam_0.R.b.x = d(0, N++); cam_0.R.b.y = d(0, N++); cam_0.R.b.z = d(0, N++);
  cam_0.x.x = d(0, N++); cam_0.x.y = d(0, N++); cam_0.x.z = d(0, N++); 
  
  cam_1.d.x = d(0, N++); cam_1.d.y = d(0, N++);
  cam_1.a.x = d(1, N++); cam_1.a.y = d(1, N++);
  cam_1.s = d(0, N++);
  cam_1.t.x = d(0, N++); cam_1.t.y = d(0, N++);
  cam_1.R.a = d(1, N++); cam_1.R.b.x = d(0, N++); cam_1.R.b.y = d(0, N++); cam_1.R.b.z = d(0, N++);
  cam_1.x.x = d(0, N++); cam_1.x.y = d(0, N++); cam_1.x.z = d(0, N++); 
  
  for (auto &set : cd.sets) {
    // If we don't know the center of the sphere for this set, we need to find it in the optimization.
    if (!set.center_valid) {
      set.center.x = d(0, N++);
      set.center.y = d(0, N++);
      set.center.z = d(0, N++);
    }
  }
  
  int it;
  for (it = 1; it <= max_iterations; it++) {
    vector3<d> baseline = cam_1.x - cam_0.x;

    // Compute J^T*J and b.
    matrix<double> JTJ(N, N);
    matrix<double> JTy(N, 1);
    for (const auto &set : cd.sets) {
      for (const auto &s : set.samples) {
        
        vector3<d> x_0 = cam_0.sensor_to_projection(s.e0, d(1.0));
        vector3<d> x_1 = cam_1.sensor_to_projection(s.e1, d(1.0));

        // z is determined by the stereo disparity.
        d z = abs(baseline)/(dot(x_0, baseline) - dot(x_1, baseline));

        // Move the points from the focal plane to the (parallel) plane containing z and add the camera origins.
        x_0 = x_0*z + cam_0.x;
        x_1 = x_1*z + cam_1.x;
                
        // Error in depth from the calibration sphere and x, for both samples.
        d r_0 = abs(x_0 - set.center) - set.radius;
        d r_1 = abs(x_1 - set.center) - set.radius;
              
        for (int i = 0; i < N; i++) {
          double Dr_0_i = D(r_0, i);
          double Dr_1_i = D(r_1, i);
          // Add this residual to J^T*y.
          JTy(i) -= Dr_0_i*r_0.f + Dr_1_i*r_1.f;
          // Add this residual to J^T*J
          for (int j = 0; j < N; j++)
            JTJ(i, j) += Dr_0_i*D(r_0, j) + Dr_1_i*D(r_1, j);
        }
      }
    }

    // Solve J^T*J*dB = J^T*y.
    matrix_ref<double> dB = solve(JTJ, JTy);
    
    if (!isfinite(dB))
      throw runtime_error("optimization diverged");

    dbg(2) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
    
    int n = 0;
    cam_0.d.x = dB(n++); cam_0.d.y = dB(n++);
    cam_0.a.x = dB(n++); cam_0.a.y = dB(n++);
    cam_0.s = dB(n++);
    cam_0.t.x = dB(n++); cam_0.t.y = dB(n++);
    cam_0.R.a = dB(n++); cam_0.R.b.x = dB(n++); cam_0.R.b.y = dB(n++); cam_0.R.b.z = dB(n++);
    cam_0.x.x = dB(n++); cam_0.x.y = dB(n++); cam_0.x.z = dB(n++); 
  
    cam_1.d.x = dB(n++); cam_1.d.y = dB(n++);
    cam_1.a.x = dB(n++); cam_1.a.y = dB(n++);
    cam_1.s = dB(n++);
    cam_1.t.x = dB(n++); cam_1.t.y = dB(n++);
    cam_1.R.a = dB(n++); cam_1.R.b.x = dB(n++); cam_1.R.b.y = dB(n++); cam_1.R.b.z = dB(n++);
    cam_1.x.x = dB(n++); cam_1.x.y = dB(n++); cam_1.x.z = dB(n++); 
  
    for (auto &set : cd.sets) {
      // If we don't know the center of the sphere for this set, we need to find them in the optimization.
      if (!set.center_valid) {
        set.center.x = dB(n++);
        set.center.y = dB(n++);
        set.center.z = dB(n++);
      }
    }
        
    if (dot(dB, dB) < epsilon_sq) {
      dbg(1) << "  converged on it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
      break;
    }
  }
    
  dump_config(cout, cam_0, cam_1);

  // Dump results to output file too.
  if (!test) {
    ofstream file(output);
    dump_config(file, cam_0, cam_1);
  }
  
  return 0;
}
