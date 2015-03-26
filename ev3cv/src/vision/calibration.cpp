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

#include <vision/calibration.h>

using namespace std;

namespace ev3cv {

namespace {

// abs'(x) is undefined at x = 0, this avoids that case.
template <typename T>
T safe_abs(const vector3<T> &x) {
  return sqrt(sqr_abs(x) + T(1e-6));
}

}

float calibrate(
    const vector<sphere_observation_set> &sphere_observations,
    cameraf &cam0f, cameraf &cam1f,
    ostream &log,
    const string &enable,
    int max_iterations, float epsilon,
    float lambda_init, float lambda_decay) {
  typedef diff<double, 26> d;

  camera<d> cam0 = camera_cast<d>(cam0f);
  camera<d> cam1 = camera_cast<d>(cam1f);

  bool enable_d1 = enable.find("d1") != string::npos;
  bool enable_a = enable.find("a") != string::npos;
  bool enable_s = enable.find("s") != string::npos;
  bool enable_c = enable.find("c") != string::npos;
  bool enable_R = enable.find("R") != string::npos;
  bool enable_x = enable.find("x") != string::npos;

  log << "Optimization variables:" << endl;

  // Construct the variables used in the optimization.
  int N = 0;
  if (enable_d1) {
    cam0.d1.x.d(N++) = 1; cam0.d1.y.d(N++) = 1;
    cam1.d1.x.d(N++) = 1; cam1.d1.y.d(N++) = 1;
    log << "  d1" << endl;
  }
  if (enable_a) {
    cam0.a.x.d(N++) = 1; cam0.a.y.d(N++) = 1;
    cam1.a.x.d(N++) = 1; cam1.a.y.d(N++) = 1;
    log << "  a" << endl;
  }
  if (enable_s) {
    cam0.s.d(N++) = 1;
    cam1.s.d(N++) = 1;
    log << "  s" << endl;
  }
  if (enable_c) {
    cam0.c.x.d(N++) = 1; cam0.c.y.d(N++) = 1;
    cam1.c.x.d(N++) = 1; cam1.c.y.d(N++) = 1;
    log << "  c" << endl;
  }
  vector3<d> R0 = vector_cast<d>(to_rodrigues(cam0f.R));
  vector3<d> R1 = vector_cast<d>(to_rodrigues(cam1f.R));
  if (enable_R) {
    R0.x.d(N++) = 1; R0.y.d(N++) = 1; R0.z.d(N++) = 1;
    R1.x.d(N++) = 1; R1.y.d(N++) = 1; R1.z.d(N++) = 1;
    log << "  R" << endl;
    cam0.R = from_rodrigues(R0);
    cam1.R = from_rodrigues(R1);
  }
  if (enable_x) {
    cam0.x.x.d(N++) = 1; cam0.x.y.d(N++) = 1; cam0.x.z.d(N++) = 1;
    cam1.x.x.d(N++) = 1; cam1.x.y.d(N++) = 1; cam1.x.z.d(N++) = 1;
    log << "  x" << endl;
  }

  vector<vector3<d>> spheres(sphere_observations.size());
  for (size_t i = 0; i < spheres.size(); i++) {
    const auto &oi = sphere_observations[i];
    auto &si = spheres[i];
    if (true /*oi.center != nullptr*/) {
      si = vector_cast<d>(oi.center);
    } else {
      si.x.d(N++) = 1;
      si.y.d(N++) = 1;
      si.z.d(N++) = 1;
      log << "  xyz" << i << endl;
    }
  }

  log << "Running optimization over " << N << " variables..." << endl;

  // Levenberg-Marquardt damping parameter.
  double lambda = lambda_init;
  double prev_error = numeric_limits<double>::infinity();
  camera<d> prev_cam0 = cam0;
  camera<d> prev_cam1 = cam1;
  vector3<d> prev_R0 = R0;
  vector3<d> prev_R1 = R1;
  vector<vector3<d>> prev_spheres = spheres;

  double error = prev_error;
  for (int it = 1; it <= max_iterations; it++) {
    d baseline = abs(cam1.x - cam0.x);
    vector3<d> b = unit(cam1.x - cam0.x);

    // Compute J^T*J and b.
    matrix<double> JTJ(N, N);
    matrix<double> JTy(N, 1);
    error = 0;

    for (size_t i = 0; i < sphere_observations.size(); i++) {
      double r_i = sphere_observations[i].radius;
      for (const auto &s : sphere_observations[i].samples) {
        vector3<d> x0 = cam0.sensor_to_projection(vector_cast<d>(s.x0), d(1.0)) - cam0.x;
        vector3<d> x1 = cam1.sensor_to_projection(vector_cast<d>(s.x1), d(1.0)) - cam1.x;

        // The camera focal planes may not be parallel to the baseline, so we tweak z
        // to make similar triangles with a vertex contained in the plane parallel
        // to the baseline.
        d z0 = abs(x0 - dot(x0, b)*b);
        d z1 = abs(x1 - dot(x1, b)*b);
        // Determine z from the adjusted focal plane positions via similar triangles.
        d z = baseline/(dot(x0, b)/z0 - dot(x1, b)/z1);

        // Project the points out to the distance z.
        x0 = x0*(z/z0) + cam0.x;
        x1 = x1*(z/z1) + cam1.x;

        // Error in depth from the calibration sphere and x, for both samples.
        d r_s0 = r_i - safe_abs(x0 - spheres[i]);
        d r_s1 = r_i - safe_abs(x1 - spheres[i]);
        error += sqr(r_s0.u);
        error += sqr(r_s1.u);

        // Error in difference between the two projected points.
        d r_z = safe_abs(x0 - x1);
        error += sqr(r_z.u);

        for (int i = 0; i < N; i++) {
          double Dr_s0_i = D(r_s0, i);
          double Dr_s1_i = D(r_s1, i);
          double Dr_z_i = D(r_z, i);
          // Add this residual to J^T*y.
          JTy(i) -= Dr_s0_i*r_s0.u;
          JTy(i) -= Dr_s1_i*r_s1.u;
          JTy(i) -= Dr_z_i*r_z.u;
          // Add this residual to J^T*J
          for (int j = 0; j < N; j++) {
            JTJ(i, j) += Dr_s0_i*D(r_s0, j);
            JTJ(i, j) += Dr_s1_i*D(r_s1, j);
            JTJ(i, j) += Dr_z_i*D(r_z, j);
          }
        }
      }
    }

    // If error increased, throw away the previous iteration and
    // reset the Levenberg-Marquardt damping parameter.
    if (error > prev_error) {
      log << "  it=" << it << ", ||dB||=<bad iteration>, error="
        << error << ", lambda=" << lambda << endl;
      lambda_init /= lambda_decay;
      lambda = lambda_init*randf(1.0f, 1.0f/lambda_decay);
      prev_error = error;
      cam0 = prev_cam0;
      cam1 = prev_cam1;
      R0 = prev_R0;
      R1 = prev_R1;
      spheres = prev_spheres;
      continue;
    } else {
      lambda_init = lambda;
    }

    // J^T*J <- J^T*J + lambda*diag(J^T*J)
    for (int i = 0; i < N; i++)
      JTJ(i, i) *= 1 + lambda;

    // Solve J^T*J*dB = J^T*y.
    matrix_ref<double> dB = solve(JTJ, JTy);

    if (!isfinite(dB))
      throw runtime_error("optimization diverged");

    log << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB))
      << ", error=" << error << ", lambda=" << lambda << endl;

    // Update Levenberg-Marquardt damping parameter.
    lambda *= lambda_decay;
    prev_error = error;
    prev_cam0 = cam0;
    prev_cam1 = cam1;
    prev_R0 = R0;
    prev_R1 = R1;
    prev_spheres = spheres;

    int n = 0;
    if (enable_d1) {
      cam0.d1.x += dB(n++); cam0.d1.y += dB(n++);
      cam1.d1.x += dB(n++); cam1.d1.y += dB(n++);
    }
    if (enable_a) {
      cam0.a.x += dB(n++); cam0.a.y += dB(n++);
      cam1.a.x += dB(n++); cam1.a.y += dB(n++);
    }
    if (enable_s) {
      cam0.s += dB(n++);
      cam1.s += dB(n++);
    }
    if (enable_c) {
      cam0.c.x += dB(n++); cam0.c.y += dB(n++);
      cam1.c.x += dB(n++); cam1.c.y += dB(n++);
    }
    if (enable_R) {
      R0.x += dB(n++); R0.y += dB(n++); R0.z += dB(n++);
      R1.x += dB(n++); R1.y += dB(n++); R1.z += dB(n++);
      cam0.R = from_rodrigues(R0);
      cam1.R = from_rodrigues(R1);
    }
    if (enable_x) {
      cam0.x.x += dB(n++); cam0.x.y += dB(n++); cam0.x.z += dB(n++);
      cam1.x.x += dB(n++); cam1.x.y += dB(n++); cam1.x.z += dB(n++);
    }

    for (size_t i = 0; i < spheres.size(); i++) {
      //const auto &oi = sphere_observations[i];
      auto &si = spheres[i];
      if (false /*oi.center == nullptr*/) {
        si.x += dB(n++);
        si.y += dB(n++);
        si.z += dB(n++);
      }
    }

    if (dot(dB, dB) < epsilon*epsilon && lambda < lambda_init*pow(lambda_decay, 5)) {
      log << "  converged on it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
      break;
    }
  }

  cam0f = camera_cast<float>(cam0);
  cam1f = camera_cast<float>(cam1);
  //for (size_t i = 0; i < sphere_observations.size(); i++)
  //  sphere_observations[i].center = vector_cast<float>(spheres[i]);
  return error;
}

namespace {

template <typename It>
void normalize_filter(It begin, It end) {
  double sum = 0.0;
  for (It i = begin; i != end; i++)
    sum += *i;
  for (It i = begin; i != end; i++)
    *i /= sum;
}

}

void filter_observations(vector<stereo_observation> &obs, float sigma) {
  // Generate a Gaussian filter.
  sigma = max(sigma, 1e-3f);
  int N = static_cast<int>(ceil(sigma*5));
  N = (N/2)*2 + 1;
  vector<float> gaussian(N);
  float sqr_sigma = sqr(sigma);
  for (int i = 0; i < N; i++) {
    float x = i - N/2;
    gaussian[i] = exp(-sqr(x)/sqr_sigma);
  }
  normalize_filter(gaussian.begin(), gaussian.end());

  // Filter the observations.
  vector<stereo_observation> obs_f;
  obs_f.reserve(obs.size() - N);
  for (int i = 0; i < static_cast<int>(obs.size()); i++) {
    stereo_observation f;
    for (int j = 0; j < N; j++) {
      int idx = clamp(i + j - N/2, 0, static_cast<int>(obs.size()));

      f.x0 += obs[idx].x0*gaussian[j];
      f.x1 += obs[idx].x1*gaussian[j];
    }
    obs_f.push_back(f);
  }

  swap(obs_f, obs);
}

float estimate_time_shift(const vector<stereo_observation> &obs) {
  // Compute the velocity of the observations.
  vector<stereo_observation> vobs;
  vobs.reserve(obs.size());
  for (size_t i = 0; i + 1 < obs.size(); i++)
    vobs.push_back({obs[i].x0 - obs[i + 1].x0, obs[i].x1 - obs[i + 1].x1});

  // Compute the L2 norm error between the sequences of velocities,
  // with +/- 1 frame of shift.
  float l2_mem[3] = { 0.0f, 0.0f, 0.0f };
  float *l2 = &l2_mem[1];
  for (int i = 1; i + 1 < static_cast<int>(vobs.size()); i++)
    for (int shift = -1; shift <= 1; shift++)
      l2[shift] += sqr_abs(vobs[i].x0 - vobs[i - shift].x1);

  // Fit a parabola to l2.
  float C = l2[0];
  float A = (l2[-1] + l2[1] - 2*C)/2;
  float B = l2[1] - A - C;

  // Check that parabola is concave up.
  if (A <= 0.0f)
    throw runtime_error("bad time shift");

  // Find the minimum of the parabola.
  float du = -B/(2*A);
  if (du < -1.0f || du > 1.0f)
    throw runtime_error("time shift was greater than +/- 1 frame");

  return du;
}

void synchronize_observations(vector<stereo_observation> &obs, float shift) {
  // Construct a Lanczos interpolation filter.
  const int lobes = 3;
  array<float, lobes*2 + 1> lcz;
  int u = static_cast<int>(floor(shift + 0.5f));
  float du = shift - u;

  for (int i = 0; i < lobes*2 + 1; i++) {
    float x = i - lobes + du;
    lcz[i] = sinc(pi*x)*sinc(pi*x/lobes);
  }
  normalize_filter(lcz.begin(), lcz.end());

  // Filter the second observation sequence.
  vector<stereo_observation> shifted;
  shifted.reserve(obs.size());
  for (int i = 0; i < static_cast<int>(obs.size()); i++) {
    stereo_observation f;
    f.x0 = obs[i].x0;
    for (int j = 0; j < static_cast<int>(lcz.size()); j++) {
      int idx = clamp(i - u + j - lobes, 0, static_cast<int>(obs.size()));
      f.x1 += obs[idx].x1*lcz[j];
    }
    shifted.push_back(f);
  }

  // Replace the observations.
  swap(shifted, obs);
}

float synchronize_observations(vector<stereo_observation> &obs) {
  float shift = estimate_time_shift(obs);
  synchronize_observations(obs, shift);
  return shift;
}

}  // namespace ev3cv
