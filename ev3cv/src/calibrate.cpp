// Copyright 2015 Google, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
#include <vision/calibration.h>
#include <ev3/nxtcam.h>

using namespace ev3cv;

static cl::group stereo_group("Camera configuration estimate");

struct camera_config {
  cl::arg<string> port;
  cl::arg<vector2i> resolution;
  cl::arg<vector2i> min, max;
  cl::arg<vector2f> distortion;

  cl::arg<float> sensor_size;
  cl::arg<float> aspect_ratio;
  cl::arg<float> focal_length;

  cl::arg<vector3f> basis_x, basis_y;
  cl::arg<vector3f> position;

  camera_config(
      const string &prefix,
      const string &port,
      const vector2i &resolution,
      const vector2i &min, const vector2i &max,
      const vector2f &distortion,
      float sensor_size, float aspect_ratio, float focal_length,
      const vector3f &basis_x,
      const vector3f &basis_y,
      const vector3f &position) :
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
  min(
    min,
    cl::name(prefix + "-min"),
    cl::desc("Minimum possible real measurement, in pixels."),
    stereo_group),
  max(
    max,
    cl::name(prefix + "-max"),
    cl::desc("Maximum possible real measurement, in pixels."),
    stereo_group),
  distortion(
    distortion,
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
  basis_x(basis_x,
    cl::name(prefix + "-basis-x"),
    cl::desc("Basis vector of the x coordinates of the camera sensor."),
    stereo_group),
  basis_y(basis_y,
    cl::name(prefix + "-basis-y"),
    cl::desc("Basis vector of the y coordinates of the camera sensor."),
    stereo_group),
  position(
    position,
    cl::name(prefix + "-position"),
    cl::desc("Position of the camera focal point."),
    stereo_group)
  {}

  cameraf to_camera() const {
    vector2f sensor_dim(aspect_ratio, 1.0f);
    sensor_dim *= sensor_size/abs(sensor_dim);
    return cameraf::from_lens(
        vector_cast<float>(*resolution),
        distortion,
        sensor_dim, focal_length,
        quaternionf::from_basis(
            unit(*basis_x),
            unit(*basis_y),
            unit(cross(*basis_x, *basis_y))),
        position);
  }
};

struct nxtcam_config : camera_config {
  nxtcam_config(
      const string &prefix,
      const string &port,
      const vector3f &basis_x,
      const vector3f &basis_y,
      const vector3f &position)
    : camera_config(
          prefix, port,
          vector2i(176, 144),
          vector2i(12, 1), vector2i(176, 143),
          vector2f(-0.05f),
          4.0f, 1.33f, 3.5f,
          basis_x, basis_y, position) {}
};

static nxtcam_config cam_config0(
    "cam0",
    "in1",
    vector3f(1.0f, 0.0f, 0.0f),
    vector3f(0.0f, 1.0f, 0.0f),
    vector3f(-5.0f, 0.0f, 0.0f));
static nxtcam_config cam_config1(
    "cam1",
    "in4",
    vector3f(1.0f, 0.0f, 0.0f),
    vector3f(0.0f, 1.0f, 0.0f),
    vector3f(5.0f, 0.0f, 0.0f));

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
static cl::arg<int> sample_distance(
  10.0f,
  cl::name("sample-distance"),
  cl::desc("Total distance overed by the object to sample, in multiples of the sensor diagonal."),
  capture_group);
static cl::arg<vector3f> sample_center(
  vector3f(0.0f),
  cl::name("sample-center"),
  cl::desc("Center of the sampling sphere."),
  capture_group);
static cl::arg<float> sample_radius(
  82.0f,
  cl::name("sample-radius"),
  cl::desc("Radius of the sampling sphere."),
  capture_group);
static cl::arg<float> sample_rate(
  30.0f,
  cl::name("sample-rate"),
  cl::desc("Frequency of camera observation samples, in Hz."),
  capture_group);

static cl::group optimization_group("Optimization parameters");

static cl::arg<int> max_iterations(
  50,
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
  0.5f,
  cl::name("lambda-decay"),
  cl::desc("Decay ratio of the Levenberg-Marquardt damping parameter on a successful iteration."),
  optimization_group);

static cl::arg<string> enable(
  "d1|a|c|R",
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

    // Turn on the cameras.
    nxtcam cam0(port_to_i2c_path(cam_config0.port));
    nxtcam cam1(port_to_i2c_path(cam_config1.port));
    cout << "Cameras:" << endl;
    cout << cam0.device_id() << " " << cam0.version() << " (" << cam0.vendor_id() << ")" << endl;
    cout << cam1.device_id() << " " << cam1.version() << " (" << cam1.vendor_id() << ")" << endl;

    cam0.track_objects();
    cam1.track_objects();
    cout << "Tracking objects..." << endl;

    // Capture samples.
    int dropped = 0;
    vector<stereo_observation> obs;
    chrono::milliseconds sample_period(static_cast<int>(1e3f/sample_rate + 0.5f));
    while (true) {
      nxtcam::blob_list blobs0 = cam0.blobs();
      nxtcam::blob_list blobs1 = cam1.blobs();

      if (blobs0.size() == 1 && blobs1.size() == 1) {
        const nxtcam::blob &b0 = blobs0.front();
        const nxtcam::blob &b1 = blobs1.front();
        int d0 = min(min(b0.x1.x - cam_config0.min->x, b0.x1.y - cam_config0.min->y),
                     min(cam_config0.max->x - b0.x2.x, cam_config0.max->y - b0.x2.y));
        int d1 = min(min(b1.x1.x - cam_config1.min->x, b1.x1.y - cam_config1.min->y),
                     min(cam_config1.max->x - b1.x2.x, cam_config1.max->y - b1.x2.y));
        int d = min(d0, d1);

        if (d > 0) {
          stereo_observation b = { b0.center(), b1.center() };
          if (d > 20)
            cout << "\x1b[0m";
          else if (d > 10)
            cout << "\x1b[30;47m";
          else
            cout << "\x1b[37;41m";
          cout << "\rx0=" << b.x0 << ", x1=" << b.x1;
          obs.push_back(b);
          dropped = 0;
        } else if (dropped++ > 3 && !obs.empty()) {
          break;
        }
      } else if (dropped++ > 3 && !obs.empty()) {
        break;
      }
      cout.flush();

      this_thread::sleep_for(sample_period);
    }
    cout << "\x1b[0m" << endl;

    cam0.stop_tracking();
    cam1.stop_tracking();
    cout << "done tracking" << endl;

    if (static_cast<int>(obs.size()) < 20 + sample_count * 5)
      throw runtime_error("not enough samples");

    // Filter, align, and save the observations.
    filter_observations(obs, 2.0f);
    synchronize_observations(obs);
    for (int i = 0; i < sample_count; i++) {
      int s = i*(obs.size() - 20)/sample_count + 10;
      set.samples.push_back(obs[s]);
    }

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
