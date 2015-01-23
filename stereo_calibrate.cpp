#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <thread>

#include "arg_port.h"
#include "debug.h"
#include "camera.h"
#include "nxtcam.h"
#include "autodiff.h"
#include "matrix.h"

#include "calibration_data.h"

using namespace std;

static cl::group stereo_group("Camera configuration estimate");

struct camera_config {
  arg_port port;
  cl::arg<vector2i> resolution;
  cl::arg<vector2f> distortion;

  cl::arg<float> sensor_size;
  cl::arg<float> aspect_ratio;
  cl::arg<float> focal_length;

  cl::arg<vector3f> x, y;
  cl::arg<vector3f> position;

  camera_config(
      const std::string &prefix,
      const ev3::port_type &port,
      const vector2i &resolution,
      float sensor_size, float aspect_ratio, float focal_length,
      const vector3f &x,
      const vector3f &y,
      const vector3f &position,
      const vector2f &distortion = vector2f(0.0f)) : 
  port(
    port,
    cl::name(prefix + "-port"),
    cl::desc("Port this camera is attached to."),
    stereo_group),
  resolution(
    resolution,
    cl::name(prefix + "-resolution"),
    cl::desc("Resolution of the cameras, in pixels."),
    stereo_group),
  distortion(
    vector2f(0.0f, 0.0f),
    cl::name(prefix + "-distortion"),
    cl::desc("Radial distortion parameters of the camera."),
    stereo_group),
  sensor_size(
    4.0f,
    cl::name(prefix + "-sensor-size"),
    cl::desc("Camera diagonal sensor size, in mm."),
    stereo_group),
  aspect_ratio(
    1.33f,
    cl::name(prefix + "-aspect-ratio"),
    cl::desc("Camera aspect ratio."),
    stereo_group),
  focal_length(
    6.5f,
    cl::name(prefix + "-focal-length"),
    cl::desc("Focal length of the camera."),
    stereo_group),
  x(x,
    cl::name(prefix + "-x"),
    cl::desc("Basis vector of the x coordinates in the camera projection space."),
    stereo_group),
  y(y,
    cl::name(prefix + "-x"),
    cl::desc("Basis vector of the x coordinates in the camera projection space."),
    stereo_group),
  position(
    position,
    cl::name(prefix + "-position"),
    cl::desc("Origin of the camera projection basis."),
    stereo_group)
  {}

  cameraf to_camera() const {
    vector2f sensor_dim(aspect_ratio, 1.0f);
    sensor_dim *= sensor_size/abs(sensor_dim);
    return cameraf(
        vector_cast<float>(*resolution), 
        distortion,
        sensor_dim, focal_length,
        quaternionf::from_basis(*x, *y, unit(cross(*x, *y))),
        position);
  }
};

static cl::arg<string> calibration_data_file(
  "calibration_data",
  cl::name("input-file"),
  cl::flags(cl::positional));
static cl::arg<string> output_file(
  "stereo_config",
  cl::name("output-file"),
  cl::flags(cl::positional));

static cl::boolean capture(
  cl::name("capture"),
  cl::desc("Capture and append new data to the calibration data set."));
static cl::arg<int> sample_count(
  32,
  cl::name("sample-count"),
  cl::desc("Number of samples to capture."));
static cl::arg<vector3f> sample_center(
  vector3f(0.0f),
  cl::name("sample-center"),
  cl::desc("Center of the sampling sphere."));
static cl::arg<float> sample_radius(
  77.5f,
  cl::name("sample-radius"),
  cl::desc("Radius of the sampling sphere."));
static cl::arg<float> sample_min_dx(
  10.0f,
  cl::name("sample-min-dx"),
  cl::desc("Minimum distance between samples."));
static cl::arg<float> sample_max_dx(
  20.0f,
  cl::name("sample-max-dx"),
  cl::desc("Maximum distance between samples."));
static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."));

static camera_config cam_config_0(
    "cam0-",
    ev3::INPUT_1,
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, cos(53.5*pi/180 + pi/2), sin(53.5*pi/180 + pi/2)),
    vector3f(-1.0f, 0.0f, 0.0f),
    vector3f(-11.0f, 13.0f, -2.0f));
static camera_config cam_config_1(
    "cam1-",
    ev3::INPUT_4,
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, -cos(53.5*pi/180 + pi/2), -sin(53.5*pi/180 + pi/2)),
    vector3f(1.0f, 0.0f, 0.0f),
    vector3f(11.0f, 13.0f, -2.0f));

static cl::arg<int> max_iterations(
  50,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."));
static cl::arg<double> epsilon(
  1e-4,
  cl::name("epsilon"),
  cl::desc("Number to consider to be zero when solving optimization problems."));

static cl::boolean test(
  cl::name("test"),
  cl::desc("Generate simulated calibration samples."));

template <typename T, int N>
std::ostream &operator << (std::ostream &os, const diff<T, N> &d) {
  return os << d.f;
}

template <typename T, int N>
std::istream &operator >> (std::istream &is, diff<T, N> &d) {
  return is >> d.f;
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

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);
  
  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);

  if (capture) {
    // Read in the existing data.
    calibration_data<float> cd;
    try {
      std::ifstream file(calibration_data_file);
      cd = read_calibration_data<float>(file);
      cout << "Read calibration data with " << cd.sets.size() << " sets, " << cd.sample_count() << " samples." << endl;
    } catch (exception &ex) {
      cout << "Warning, failed to read calibration data file '" << *calibration_data_file << "'" << endl;
    }
    
    // Add a new dataset.
    cd.sets.emplace_back();
    calibration_data<float>::set &set = cd.sets.back();
    set.radius = sample_radius;
    if (sample_center.parsed()) {
      set.center = sample_center;
      set.center_valid = true;
    }

    // Turn on the cameras.
    nxtcam cam_0(cam_config_0.port);
    nxtcam cam_1(cam_config_1.port);
    cout << "Cameras:" << endl;
    cout << cam_0.device_id() << " " << cam_0.version() << " (" << cam_0.vendor_id() << ")" << endl;
    cout << cam_1.device_id() << " " << cam_1.version() << " (" << cam_1.vendor_id() << ")" << endl;

    cam_0.track_objects();
    cam_1.track_objects();

    // t will increment in regular intervals of T.
    typedef chrono::steady_clock clock;
    auto t = clock::now();
    chrono::milliseconds T(static_cast<int>(1e3f/sample_rate + 0.5f));
  
    // Capture samples.
    while (static_cast<int>(set.samples.size()) < sample_count) {
      nxtcam::blob_list blobs0 = cam_0.blobs();
      nxtcam::blob_list blobs1 = cam_1.blobs();

      if (blobs0.size() == 1 && blobs1.size() == 1) {
        const nxtcam::blob &b0 = blobs0.front();
        const nxtcam::blob &b1 = blobs1.front();

        if(set.samples.empty()) {
          set.samples.emplace_back(b0.center(), b1.center());
          cout << "Sample " << set.samples.size() << ": " << b0.center() << ", " << b1.center() << endl;
        } else {
          float dx = max(abs(b0.center() - set.samples.back().e0), abs(b1.center() - set.samples.back().e1));
          if (sample_min_dx <= dx && dx < sample_max_dx) {
            set.samples.emplace_back(b0.center(), b1.center());
            cout << "Sample " << set.samples.size() << ": " << b0.center() << ", " << b1.center() << endl;
          }
        }
      }

      t += T;
      this_thread::sleep_until(t);
    }

    cam_0.stop_tracking();
    cam_1.stop_tracking();

    // Write out the updated calibration data.
    std::ofstream file(calibration_data_file);
    write_calibration_data(file, cd);

  } else {
    typedef diff<double, 34> d;

    std::ifstream file(calibration_data_file);
    calibration_data<d> cd = read_calibration_data<d>(file);

    if (cd.sets.empty()) 
      throw runtime_error("no calibration data");

    // Get a list of samples corresponding to observations of an object lying somewhere on the sampling sphere. 
    const double epsilon_sq = epsilon*epsilon;
 
    camera<d> cam_0 = camera_cast<d>(cam_config_0.to_camera());
    camera<d> cam_1 = camera_cast<d>(cam_config_1.to_camera());

    // Construct the variables used in the optimization.
    int N = 0;
    cam_0.d.x.df[N++] = 1; cam_0.d.y.df[N++] = 1;
    cam_0.a.x.df[N++] = 1; cam_0.a.y.df[N++] = 1;
    cam_0.s.df[N++] = 1;
    cam_0.t.x.df[N++] = 1; cam_0.t.y.df[N++] = 1;
    cam_0.R.a.df[N++] = 1; cam_0.R.b.x.df[N++] = 1; cam_0.R.b.y.df[N++] = 1; cam_0.R.b.z.df[N++] = 1;
  
    cam_1.d.x.df[N++] = 1; cam_1.d.y.df[N++] = 1;
    cam_1.a.x.df[N++] = 1; cam_1.a.y.df[N++] = 1;
    cam_1.s.df[N++] = 1;
    cam_1.t.x.df[N++] = 1; cam_1.t.y.df[N++] = 1;
    cam_1.R.a.df[N++] = 1; cam_1.R.b.x.df[N++] = 1; cam_1.R.b.y.df[N++] = 1; cam_1.R.b.z.df[N++] = 1;

    for (auto &set : cd.sets) {
      // If we don't know the center of the sphere for this set, we need to find it in the optimization.
      if (!set.center_valid) {
        set.center.x.df[N++] = 1;
        set.center.y.df[N++] = 1;
        set.center.z.df[N++] = 1;
      }
    }
  
    int it;
    for (it = 1; it <= max_iterations; it++) {
      d baseline = abs(cam_1.x - cam_0.x);
      vector3<d> b = unit(cam_1.x - cam_0.x);
    
      // Compute J^T*J and b.
      matrix<double> JTJ(N, N);
      matrix<double> JTy(N, 1);
    
      for (const auto &set : cd.sets) {
        for (const auto &s : set.samples) {
          vector3<d> x_0 = cam_0.sensor_to_projection(s.e0, d(1.0)) - cam_0.x;
          vector3<d> x_1 = cam_1.sensor_to_projection(s.e1, d(1.0)) - cam_1.x;

          // z is determined by the stereo disparity.
          d z = baseline/(dot(x_0, b) - dot(x_1, b));

          // Move the points from the focal plane to the (parallel) plane containing z and add the camera origins.
          x_0 = x_0*z + cam_0.x;
          x_1 = x_1*z + cam_1.x;

          // Error in depth from the calibration sphere and x, for both samples.
          d r_0 = set.radius - abs(x_0 - set.center);
          d r_1 = set.radius - abs(x_1 - set.center);
        
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
    
      dbg(2) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB));
    
      int n = 0;
      cam_0.d.x += dB(n++); cam_0.d.y += dB(n++);
      cam_0.a.x += dB(n++); cam_0.a.y += dB(n++);
      cam_0.s += dB(n++);
      cam_0.t.x += dB(n++); cam_0.t.y += dB(n++);
      cam_0.R.a += dB(n++); cam_0.R.b.x += dB(n++); cam_0.R.b.y += dB(n++); cam_0.R.b.z += dB(n++);
  
      cam_1.d.x += dB(n++); cam_1.d.y += dB(n++);
      cam_1.a.x += dB(n++); cam_1.a.y += dB(n++);
      cam_1.s += dB(n++);
      cam_1.t.x += dB(n++); cam_1.t.y += dB(n++);
      cam_1.R.a += dB(n++); cam_1.R.b.x += dB(n++); cam_1.R.b.y += dB(n++); cam_1.R.b.z += dB(n++);

      // Renormalize quaternions.
      cam_0.R /= abs(cam_0.R);
      cam_1.R /= abs(cam_1.R);
  
      for (auto &i : cd.sets) {
        // If we don't know the center of the sphere for this set, we need to find them in the optimization.
        if (!i.center_valid) {
          i.center.x += dB(n++);
          i.center.y += dB(n++);
          i.center.z += dB(n++);
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
      ofstream file(output_file);
      dump_config(file, cam_0, cam_1);
    }
  }  

  return 0;
}
