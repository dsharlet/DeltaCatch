#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace std;

#include <cl/cl.h>
#include <vision/camera.h>
#include <vision/nxtcam.h>
#include <vision/calibration.h>

using namespace ev3cv;

static cl::group stereo_group("Camera configuration estimate");

struct camera_config {
  cl::arg<string> port;
  cl::arg<vector2i> resolution;
  cl::arg<vector2f> distortion;

  cl::arg<float> sensor_size;
  cl::arg<float> aspect_ratio;
  cl::arg<float> focal_length;

  cl::arg<vector3f> x, y;
  cl::arg<vector3f> position;

  camera_config(
      const string &prefix,
      const string &port,
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

static camera_config cam_config0(
    "cam0",
    "in1",
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, -cos(53.5*pi/180 + pi/2), -sin(53.5*pi/180 + pi/2)),
    vector3f(1.0f, 0.0f, 0.0f),
    vector3f(-11.15f, 12.5f, -3.0f));
static camera_config cam_config1(
    "cam1",
    "in4",
    vector2i(176, 144),
    4.0f, 1.33f, 3.5f,
    vector3f(0.0f, cos(53.5*pi/180 + pi/2), sin(53.5*pi/180 + pi/2)),
    vector3f(-1.0f, 0.0f, 0.0f),
    vector3f(11.15f, 12.5f, -3.0f));

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
  20.0f,
  cl::name("sample-min-dx"),
  cl::desc("Minimum distance between samples."),
  capture_group);
static cl::arg<float> sample_noise_tolerance(
  1.0f,
  cl::name("sample-noise-tolerance"),
  cl::desc("Maximum allowed error in a set of observations to be taken as a sample."),
  capture_group);
static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."),
  capture_group);

static cl::arg<vector2i> cam0_min(
  vector2i(12, 1),
  cl::name("cam0-min"),
  cl::desc("Minimum real measurement from camera 0."));
static cl::arg<vector2i> cam1_min(
  vector2i(12, 1),
  cl::name("cam1-min"),
  cl::desc("Minimum real measurement from camera 1."));

static cl::arg<vector2i> cam0_max(
  vector2i(176, 143),
  cl::name("cam0-max"),
  cl::desc("Maximum real measurement from camera 0."));
static cl::arg<vector2i> cam1_max(
  vector2i(176, 143),
  cl::name("cam1-max"),
  cl::desc("Maximum real measurement from camera 1."));

static cl::group optimization_group("Optimization parameters");

static cl::arg<int> max_iterations(
  100,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."),
  optimization_group);
static cl::arg<float> convergence_threshold(
  1e-3f,
  cl::name("convergence-threshold"),
  cl::desc("Smallest improvement in residual error before optimization is considered to be converged."),
  optimization_group);
static cl::arg<float> lambda_init(
  1.0f,
  cl::name("lambda-init"),
  cl::desc("Initial value of Levenberg-Marquardt damping parameter."),
  optimization_group);
static cl::arg<float> lambda_decay(
  0.9f,
  cl::name("lambda-decay"),
  cl::desc("Decay ratio of the Levenberg-Marquardt damping parameter on a successful iteration."),
  optimization_group);

static cl::arg<string> enable(
  "d1|a|t|R",
  cl::name("enable"),
  cl::desc("Which calibration parameters to allow optimization over."),
  optimization_group);

void write_sphere_observations(
    ostream &os, 
    const vector<sphere_observation_set> &spheres) {
  for (const auto &sphere : spheres) {
    os << "set " << sphere.radius << " " << sphere.center << endl;
    for (const auto &s : sphere.samples)
      os << "sample " << s.x0 << " " << s.x1 << endl;
  }
}

vector<sphere_observation_set> read_sphere_observations(istream &is) {
  vector<sphere_observation_set> spheres;
  while (is.good()) {
    string line_buf;
    getline(is, line_buf);
    stringstream line(line_buf);

    string cmd;
    line >> cmd;
    if (cmd == "set") {
      sphere_observation_set sphere;
      line >> sphere.radius >> sphere.center;
      spheres.push_back(sphere);
    } else if (cmd == "sample") {
      if (spheres.empty())
        throw runtime_error("calibration data missing set descriptor");
      stereo_observation s;
      line >> s.x0 >> s.x1;
      if (!line.bad())
        spheres.back().samples.push_back(s);
    }
  }
  return spheres;
}

void dump_config(ostream &os, const string &prefix, const cameraf &cam) {
  os << prefix << "resolution " << cam.resolution << endl;
  os << prefix << "distortion " << cam.d1 << endl;
  os << prefix << "calibration " << cam.K() << endl;
  os << prefix << "orientation " << cam.R << endl;
  os << prefix << "position " << cam.x << endl;
}

void dump_config(ostream &os, const string &prefix, cameraf &cam0, const cameraf &cam1) {
  dump_config(os, prefix + "cam0-", cam0);
  dump_config(os, prefix + "cam1-", cam1);
}

template <size_t N>
vector2f mean(const circular_array<vector2f, N> &x) {
  vector2f mu(0.0f);
  for (size_t i = x.begin(); i != x.end(); i++)
    mu += x[i];
  return mu/x.size();
}

template <size_t N>
pair<vector2f, vector2f> min_max(const circular_array<vector2f, N> &x) {
  vector2f a(numeric_limits<float>::infinity());
  vector2f b(-numeric_limits<float>::infinity());
  for (size_t i = x.begin(); i != x.end(); i++) {
    a.x = min(a.x, x[i].x);
    a.y = min(a.y, x[i].y);
    b.x = max(b.x, x[i].x);
    b.y = max(b.y, x[i].y);
  }
  return make_pair(a, b);
}

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);
  
  // Reduce clutter of insignificant digits.
  cout << fixed << showpoint << setprecision(3);
  cerr << fixed << showpoint << setprecision(3);

  if (capture) {
    // Read in the existing data.
    vector<sphere_observation_set> spheres;
    try {
      ifstream input(calibration_data_file);
      spheres = read_sphere_observations(input);
      cout << "Read calibration data with " << spheres.size() << " sets" << endl;
    } catch (exception &ex) {
      cout << "Warning, failed to read calibration data file '" << *calibration_data_file << "'" << endl;
    }
    
    // Add a new dataset.
    spheres.emplace_back();
    sphere_observation_set &set = spheres.back();
    set.radius = sample_radius;
    set.center = *sample_center;
    set.samples.reserve(sample_count);

    // Turn on the cameras.
    nxtcam cam0(port_to_i2c_path(cam_config0.port));
    nxtcam cam1(port_to_i2c_path(cam_config1.port));
    cout << "Cameras:" << endl;
    cout << cam0.device_id() << " " << cam0.version() << " (" << cam0.vendor_id() << ")" << endl;
    cout << cam1.device_id() << " " << cam1.version() << " (" << cam1.vendor_id() << ")" << endl;

    cam0.track_objects();
    cam1.track_objects();
    cout << "Tracking objects..." << endl;

    chrono::milliseconds sample_period(static_cast<int>(1e3f/sample_rate + 0.5f));

    circular_array<vector2f, 4> x0, x1;

    // Capture samples.
    while (static_cast<int>(set.samples.size()) < sample_count) {
      nxtcam::blob_list blobs0 = cam0.blobs();
      nxtcam::blob_list blobs1 = cam1.blobs();
      
      if (blobs0.size() == 1 && blobs1.size() == 1) {
        const nxtcam::blob &b0 = blobs0.front();
        const nxtcam::blob &b1 = blobs1.front();
        if (b0.x1.x <= cam0_min->x || b0.x1.y <= cam0_min->y ||
            b0.x2.x >= cam0_max->x || b0.x2.y >= cam0_max->y)
          continue;
        if (b1.x1.x <= cam1_min->x || b1.x1.y <= cam1_min->y ||
            b1.x2.x >= cam1_max->x || b1.x2.y >= cam1_max->y)
          continue;

        while(x0.size() >= x0.capacity()) x0.pop_front();
        while(x1.size() >= x1.capacity()) x1.pop_front();
        x0.push_back(b0.center());
        x1.push_back(b1.center());

        vector2f x0_mean = mean(x0);
        vector2f x1_mean = mean(x1);

        cout << "\rn=" << set.samples.size() << ", x0=" << x0_mean << ", x1=" << x1_mean << "                  ";
        cout.flush();

        // Only take a sample if we have a full set of observations to filter.
        if (x0.size() >= x0.capacity() || x1.size() >= x1.capacity()) {
          // Only take a sample if the min and max of the observations in the buffer are within noise tolerances.
          vector2f x0_min, x0_max;
          vector2f x1_min, x1_max;
          tie(x0_min, x0_max) = min_max(x0);
          tie(x1_min, x1_max) = min_max(x1);
          if (abs(x0_max - x0_min) < sample_noise_tolerance &&
              abs(x1_max - x1_min) < sample_noise_tolerance) {
            if(set.samples.empty()) {
              set.samples.push_back({x0_mean, x1_mean});
              cout << endl;
            } else { 
              float dx = max(abs(x0_mean - set.samples.back().x0), abs(x1_mean - set.samples.back().x1));
              if (sample_min_dx <= dx) {
                set.samples.push_back({x0_mean, x1_mean});
                cout << endl;
              }
            }
          }
        }
      } else {
        x0.clear();
        x1.clear();
        cout << "\r" << string(79, ' ');
        cout.flush();
      }

      this_thread::sleep_for(sample_period);
    }

    cam0.stop_tracking();
    cam1.stop_tracking();

    // Write out the updated calibration data.
    ofstream output(calibration_data_file);
    write_sphere_observations(output, spheres);
  } else {
    ifstream input(calibration_data_file);
    vector<sphere_observation_set> spheres = read_sphere_observations(input);

    if (spheres.empty()) 
      throw runtime_error("no calibration data");
    cout << "Read calibration data with " << spheres.size() << " sets" << endl;
    
    cameraf cam0 = cam_config0.to_camera();
    cameraf cam1 = cam_config1.to_camera();

    calibrate(
        spheres, 
        cam0, cam1, 
        cout, 
        enable, 
        max_iterations, convergence_threshold, 
        lambda_init, lambda_decay);

    dump_config(cout, "   ", cam0, cam1);
        
    // Dump results to output file too.
    ofstream output(output_file);
    dump_config(output, "--", cam0, cam1);
  }  

  return 0;
}
