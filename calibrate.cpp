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
    1.0f,
    cl::name(prefix + "-focal-length"),
    cl::desc("Focal length of the camera."),
    stereo_group),
  x(x,
    cl::name(prefix + "-x"),
    cl::desc("Basis vector of the x coordinates of the camera sensor."),
    stereo_group),
  y(y,
    cl::name(prefix + "-y"),
    cl::desc("Basis vector of the y coordinates of the camera sensor."),
    stereo_group),
  position(
    position,
    cl::name(prefix + "-position"),
    cl::desc("Position of the camera focal point."),
    stereo_group)
  {}

  cameraf to_camera() const {
    vector2f sensor_dim(aspect_ratio, -1.0f);
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

static cl::group capture_group("Data capture");

static cl::boolean capture(
  cl::name("capture"),
  cl::desc("Capture and append a new set of data to the calibration data."),
  capture_group);
static cl::arg<int> sample_count(
  32,
  cl::name("sample-count"),
  cl::desc("Number of samples to capture."),
  capture_group);
static cl::arg<vector3f> sample_center(
  vector3f(0.0f),
  cl::name("sample-center"),
  cl::desc("Center of the sampling sphere."),
  capture_group);
static cl::arg<float> sample_radius(
  77.5f,
  cl::name("sample-radius"),
  cl::desc("Radius of the sampling sphere."),
  capture_group);
static cl::arg<float> sample_min_dx(
  10.0f,
  cl::name("sample-min-dx"),
  cl::desc("Minimum distance between samples."),
  capture_group);
static cl::arg<float> sample_max_dx(
  20.0f,
  cl::name("sample-max-dx"),
  cl::desc("Maximum distance between samples."),
  capture_group);
static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."),
  capture_group);

static camera_config cam_config0(
    "cam0",
    ev3::INPUT_1,
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, -cos(53.5*pi/180 + pi/2), -sin(53.5*pi/180 + pi/2)),
    vector3f(1.0f, 0.0f, 0.0f),
    vector3f(-11.15f, 12.5f, -3.0f));
static camera_config cam_config1(
    "cam1",
    ev3::INPUT_4,
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, cos(53.5*pi/180 + pi/2), sin(53.5*pi/180 + pi/2)),
    vector3f(-1.0f, 0.0f, 0.0f),
    vector3f(11.15f, 12.5f, -3.0f));

static cl::group optimization_group("Optimization parameters");

static cl::arg<int> max_iterations(
  50,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."),
  optimization_group);
static cl::arg<double> epsilon(
  1e-4,
  cl::name("epsilon"),
  cl::desc("Number to consider to be zero when solving optimization problems."),
  optimization_group);
static cl::arg<double> lambda_recovery(
  1.0,
  cl::name("lambda-init"),
  cl::desc("Initial value of Levenberg-Marquardt damping parameter."),
  optimization_group);
static cl::arg<double> lambda_decay(
  0.5,
  cl::name("lambda-decay"),
  cl::desc("Decay ratio of the Levenberg-Marquardt damping parameter on a successful iteration."),
  optimization_group);

static cl::arg<string> enable(
  "dastR",
  cl::name("enable"),
  cl::desc("Which calibration parameters to allow optimization over."),
  optimization_group);

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
  os << prefix << "resolution " << cam.resolution << endl;
  os << prefix << "distortion " << cam.d << endl;
  os << prefix << "calibration " << cam.K() << endl;
  os << prefix << "orientation " << cam.R << endl;
  os << prefix << "origin " << cam.x << endl;
}

template <typename T>
void dump_config(ostream &os, const camera<T> &cam0, const camera<T> &cam1) {
  dump_config(os, "--cam0-", cam0);
  dump_config(os, "--cam1-", cam1);
}

template <typename T>
vector<vector3<T>> unknown_centers(const calibration_data<T> &cd) {
  vector<vector3<T>> c;
  c.reserve(cd.sets.size());
  for (const auto &i : cd.sets)
    if (!i.center_valid)
      c.push_back(i.center);
  return c;
}

template <typename T>
void set_unknown_centers(calibration_data<T> &cd, const vector<vector3<T>> &centers) {
  typename vector<vector3<T>>::const_iterator c = centers.begin();
  for (auto &i : cd.sets)
    if (!i.center_valid)
      i.center = *c++;
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
      ifstream input(calibration_data_file);
      cd = read_calibration_data<float>(input);
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
    set.samples.reserve(sample_count);

    // Turn on the cameras.
    nxtcam cam0(cam_config0.port);
    nxtcam cam1(cam_config1.port);
    cout << "Cameras:" << endl;
    cout << cam0.device_id() << " " << cam0.version() << " (" << cam0.vendor_id() << ")" << endl;
    cout << cam1.device_id() << " " << cam1.version() << " (" << cam1.vendor_id() << ")" << endl;

    cam0.track_objects();
    cam1.track_objects();
    cout << "Tracking objects..." << endl;

    chrono::milliseconds sample_period(static_cast<int>(1e3f/sample_rate + 0.5f));
  
    // Capture samples.
    while (static_cast<int>(set.samples.size()) < sample_count) {
      nxtcam::blob_list blobs0 = cam0.blobs();
      nxtcam::blob_list blobs1 = cam1.blobs();

      if (blobs0.size() == 1 && blobs1.size() == 1) {
        vector2f x0 = blobs0.front().center();
        vector2f x1 = blobs1.front().center();

        if(set.samples.empty()) {
          set.samples.emplace_back(x0, x1);
          cout << "Sample " << set.samples.size() << ": " << x0 << ", " << x1 << endl;
        } else {
          float dx = max(abs(x0 - set.samples.back().px0), abs(x1 - set.samples.back().px1));
          if (sample_min_dx <= dx && dx < sample_max_dx) {
            set.samples.emplace_back(x0, x1);
            cout << "Sample " << set.samples.size() << ": " << x0 << ", " << x1 << endl;
          }
        }
      }

      this_thread::sleep_for(sample_period);
    }

    cam0.stop_tracking();
    cam1.stop_tracking();

    // Write out the updated calibration data.
    ofstream output(calibration_data_file);
    write_calibration_data(output, cd);
  } else {
    typedef diff<double, 40> d;

    ifstream input(calibration_data_file);
    calibration_data<d> cd = read_calibration_data<d>(input);

    if (cd.sets.empty()) 
      throw runtime_error("no calibration data");

    // Get a list of samples corresponding to observations of an object lying somewhere on the sampling sphere. 
    const double epsilon_sq = epsilon*epsilon;
 
    camera<d> cam0 = camera_cast<d>(cam_config0.to_camera());
    camera<d> cam1 = camera_cast<d>(cam_config1.to_camera());

    bool enable_d = enable->find('d') != string::npos;
    bool enable_a = enable->find('a') != string::npos;
    bool enable_s = enable->find('s') != string::npos;
    bool enable_t = enable->find('t') != string::npos;
    bool enable_R = enable->find('R') != string::npos;

    // Construct the variables used in the optimization.
    int N = 0;
    if (enable_d) {
      cam0.d.x.df[N++] = 1; cam0.d.y.df[N++] = 1;
      cam1.d.x.df[N++] = 1; cam1.d.y.df[N++] = 1;
    }
    if (enable_a) {
      cam0.a.x.df[N++] = 1; cam0.a.y.df[N++] = 1;
      cam1.a.x.df[N++] = 1; cam1.a.y.df[N++] = 1;
    }
    if (enable_s) {
      cam0.s.df[N++] = 1;
      cam1.s.df[N++] = 1;
    }
    if (enable_t) {
      cam0.t.x.df[N++] = 1; cam0.t.y.df[N++] = 1;
      cam1.t.x.df[N++] = 1; cam1.t.y.df[N++] = 1;
    }
    if (enable_R) {
      cam0.R.a.df[N++] = 1; cam0.R.b.x.df[N++] = 1; cam0.R.b.y.df[N++] = 1; cam0.R.b.z.df[N++] = 1;
      cam1.R.a.df[N++] = 1; cam1.R.b.x.df[N++] = 1; cam1.R.b.y.df[N++] = 1; cam1.R.b.z.df[N++] = 1;
    }

    for (auto &set : cd.sets) {
      // If we don't know the center of the sphere for this set, we need to find it in the optimization.
      if (!set.center_valid) {
        set.center.x.df[N++] = 1;
        set.center.y.df[N++] = 1;
        set.center.z.df[N++] = 1;
      }
    }
  
    // Levenberg-Marquardt damping parameter.
    double lambda = lambda_recovery/lambda_decay;
    double prev_error = numeric_limits<double>::infinity();
    camera<d> prev_cam0 = cam0;
    camera<d> prev_cam1 = cam1;
    vector<vector3<d>> prev_centers = unknown_centers(cd);

    int it;
    for (it = 1; it <= max_iterations; it++) {
      d baseline = abs(cam1.x - cam0.x);
      vector3<d> b = unit(cam1.x - cam0.x);
    
      double error = 0;

      // Compute J^T*J and b.
      matrix<double> JTJ(N, N);
      matrix<double> JTy(N, 1);
    
      for (const auto &set : cd.sets) {
        for (const auto &s : set.samples) {
          vector3<d> x0 = cam0.sensor_to_projection(s.px0, d(1.0)) - cam0.x;
          vector3<d> x1 = cam1.sensor_to_projection(s.px1, d(1.0)) - cam1.x;

          // z is determined by the stereo disparity.
          d z = baseline/(dot(x0, b) - dot(x1, b));

          // Move the points from the focal plane to the (parallel) plane containing z and add the camera origins.
          x0 = x0*z + cam0.x;
          x1 = x1*z + cam1.x;

          // Error in depth from the calibration sphere and x, for both samples.
          d r_s = set.radius - abs((x0 + x1)/2 - set.center);
          error += sqr(r_s.f);
        
          // Error in difference between the two projected points.
          d r_z = abs(x0 - x1);
          error += sqr(r_z.f);

          for (int i = 0; i < N; i++) {
            double Dr_s_i = D(r_s, i);
            double Dr_z_i = D(r_z, i);
            // Add this residual to J^T*y.
            JTy(i) -= Dr_s_i*r_s.f + Dr_z_i*r_z.f;
            // Add this residual to J^T*J
            for (int j = 0; j < N; j++)
              JTJ(i, j) += Dr_s_i*D(r_s, j) + Dr_z_i*D(r_z, j);
          }
        }
      }

      // Update Levenberg-Marquardt damping parameter.
      if (error < prev_error) {
        lambda *= lambda_decay;
        prev_error = error;
        prev_cam0 = cam0;
        prev_cam1 = cam1;
        prev_centers = unknown_centers(cd);
      } else {
        lambda = lambda_recovery/lambda_decay;
        prev_error = error;
        cam0 = prev_cam0;
        cam1 = prev_cam1;
        set_unknown_centers(cd, prev_centers);
        dbg(2) << "  it=" << it << ", ||dB||=0, error=" << error << ", lambda=" << lambda << endl;
        continue;
      }

      // J^T*J <- J^J*J + lambda*diag(J^J*J)
      for (int i = 0; i < N; i++)
        JTJ(i, i) *= 1 + lambda;

      // Solve J^T*J*dB = J^T*y.
      matrix_ref<double> dB = solve(JTJ, JTy);
    
      if (!isfinite(dB))
        throw runtime_error("optimization diverged");
    
      dbg(2) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) 
        << ", error=" << error << ", lambda=" << lambda << endl;
    
      int n = 0;
      if (enable_d) {
        cam0.d.x += dB(n++); cam0.d.y += dB(n++);
        cam1.d.x += dB(n++); cam1.d.y += dB(n++);
      }
      if (enable_a) {
        cam0.a.x += dB(n++); cam0.a.y += dB(n++);
        cam1.a.x += dB(n++); cam1.a.y += dB(n++);
      }
      if (enable_s) {
        cam0.s += dB(n++);
        cam1.s += dB(n++);
      }
      if (enable_t) {
        cam0.t.x += dB(n++); cam0.t.y += dB(n++);
        cam1.t.x += dB(n++); cam1.t.y += dB(n++);
      }
      if (enable_R) {
        cam0.R.a += dB(n++); cam0.R.b.x += dB(n++); cam0.R.b.y += dB(n++); cam0.R.b.z += dB(n++);
        cam1.R.a += dB(n++); cam1.R.b.x += dB(n++); cam1.R.b.y += dB(n++); cam1.R.b.z += dB(n++);
        // Renormalize quaternions.
        cam0.R /= abs(cam0.R);
        cam1.R /= abs(cam1.R);
      }
  
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
    
    dump_config(cout, cam0, cam1);

    for (size_t i = 0; i < cd.sets.size(); i++) {
      if (cd.sets[i].center_valid)
        cout << "Known sampling sphere ";
      else
        cout << "Estimated sampling sphere ";
      cout << i << " center=" << cd.sets[i].center << ", radius=" << cd.sets[i].radius << endl;
    }
    
    // Dump results to output file too.
    ofstream output(output_file);
    dump_config(output, cam0, cam1);
  }  

  return 0;
}
