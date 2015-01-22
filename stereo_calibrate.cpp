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

static cl::group stereo_group("Camera configuration estimate");

struct camera_config {
  cl::arg<vector2i> resolution;
  cl::arg<vector2f> distortion;

  cl::arg<float> sensor_size;
  cl::arg<float> aspect_ratio;
  cl::arg<float> focal_length;

  cl::arg<vector3f> x, y;
  cl::arg<vector3f> position;

  camera_config(
      const std::string &prefix,
      const vector2i &resolution,
      float sensor_size, float aspect_ratio, float focal_length,
      const vector3f &x,
      const vector3f &y,
      const vector3f &position,
      const vector2f &distortion = vector2f(0.0f)) : 
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

static camera_config cam_config_0(
    "cam0-",
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, cos(53.5*pi/180 + pi/2), sin(53.5*pi/180 + pi/2)),
    vector3f(-1.0f, 0.0f, 0.0f),
    vector3f(-11.0f, 13.0f, -2.0f));
static camera_config cam_config_1(
    "cam1-",
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, -cos(53.5*pi/180 + pi/2), -sin(53.5*pi/180 + pi/2)),
    vector3f(1.0f, 0.0f, 0.0f),
    vector3f(11.0f, 13.0f, -2.0f));

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

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);
  
  // Get a list of samples corresponding to observations of an object lying somewhere on the sampling sphere. 
  const double epsilon_sq = epsilon*epsilon;
  
  typedef diff<double, 40> d;
  calibration_data<d> cd;

  camera<d> cam_0 = camera_cast<d>(cam_config_0.to_camera());
  camera<d> cam_1 = camera_cast<d>(cam_config_1.to_camera());
  
  cam_0.d = vector_cast<d>(randv2f()*1e-1f);
  cam_1.d = vector_cast<d>(randv2f()*1e-1f);
  //cam_0.t = vector_cast<d>(randv2f(-1.0f, 1.0f));
  //cam_1.t = vector_cast<d>(randv2f(-1.0f, 1.0f));

  dump_config(cout, cam_0, cam_1);

  float r = 60.0f;
  for (int i = 0; i < 5; i++) {
    cd.sets.emplace_back();
    calibration_data<d>::set &set = cd.sets.back();
    if (i == 0) {
      set.center = vector3<d>(0, 0, 6);
      set.radius = r;
      set.center_valid = true;
    } else {
      set.center = vector_cast<d>(randv3f(vector3f(-20.0f, -20.0f, -10.0f), vector3f(20.0f, 20.0f, 20.0f)));
      set.radius = r;
      set.center_valid = false;
    }
    while(set.samples.size() < 20) {
      vector3<d> x = vector_cast<d>(unit(randv3f(-1.0f, 1.0f)))*set.radius + set.center;
      if (cam_0.is_visible(x) && cam_1.is_visible(x)) {
        set.samples.push_back(calibration_data<d>::sample(cam_0.project_to_sensor(x), cam_1.project_to_sensor(x)));
      }
    }
  }
  
  cam_0 = camera_cast<d>(cam_config_0.to_camera());
  cam_1 = camera_cast<d>(cam_config_1.to_camera());

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
    
    dbg(2) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
    
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
    ofstream file(output);
    dump_config(file, cam_0, cam_1);
  }
  
  return 0;
}
