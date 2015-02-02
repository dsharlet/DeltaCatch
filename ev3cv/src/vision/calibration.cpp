#include <vision/calibration.h>

using namespace std;

namespace ev3cv {

float calibrate(
    const vector<sphere_observation_set> &sphere_observations,
    cameraf &cam0f, cameraf &cam1f,
    ostream &log,
    const string &enable,
    float lambda_init, float lambda_decay,
    float epsilon, int max_iterations) {

  typedef diff<double, 28> d;
   
  camera<d> cam0 = camera_cast<d>(cam0f);
  camera<d> cam1 = camera_cast<d>(cam1f);

  bool enable_d1 = enable.find("d1") != string::npos;
  bool enable_a = enable.find("a") != string::npos;
  bool enable_s = enable.find("s") != string::npos;
  bool enable_t = enable.find("t") != string::npos;
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
  if (enable_t) {
    cam0.t.x.d(N++) = 1; cam0.t.y.d(N++) = 1;
    cam1.t.x.d(N++) = 1; cam1.t.y.d(N++) = 1;
    log << "  t" << endl;
  }
  if (enable_R) {
    cam0.R.a.d(N++) = 1; cam0.R.b.x.d(N++) = 1; cam0.R.b.y.d(N++) = 1; cam0.R.b.z.d(N++) = 1;
    cam1.R.a.d(N++) = 1; cam1.R.b.x.d(N++) = 1; cam1.R.b.y.d(N++) = 1; cam1.R.b.z.d(N++) = 1;
    log << "  R" << endl;
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
  
  log << "Running optimization..." << endl;

  // Levenberg-Marquardt damping parameter.
  double lambda = lambda_init;
  double prev_error = numeric_limits<double>::infinity();
  camera<d> prev_cam0 = cam0;
  camera<d> prev_cam1 = cam1;
  vector<vector3<d>> prev_spheres = spheres;

  for (int it = 1; it <= max_iterations; it++) {
    d baseline = abs(cam1.x - cam0.x);
    vector3<d> b = unit(cam1.x - cam0.x);
    
    double error = 0;

    // Compute J^T*J and b.
    matrix<double> JTJ(N, N);
    matrix<double> JTy(N, 1);
    
    for (size_t i = 0; i < sphere_observations.size(); i++) {
      for (const auto &s : sphere_observations[i].samples) {
        vector3<d> x0 = cam0.sensor_to_projection(vector_cast<d>(s.x0), d(1.0)) - cam0.x;
        vector3<d> x1 = cam1.sensor_to_projection(vector_cast<d>(s.x1), d(1.0)) - cam1.x;

        // z is determined by the stereo disparity.
        d z = baseline/(dot(x0, b) - dot(x1, b));

        // Move the points from the focal plane to the (parallel) plane containing z and add the camera origins.
        x0 = x0*z + cam0.x;
        x1 = x1*z + cam1.x;

        // Error in depth from the calibration sphere and x, for both samples.
        d r_s0 = scalar_cast<d>(sphere_observations[i].radius) - abs(x0 - spheres[i]);
        d r_s1 = scalar_cast<d>(sphere_observations[i].radius) - abs(x1 - spheres[i]);
        error += sqr(r_s0.f);
        error += sqr(r_s1.f);
        
        // Error in difference between the two projected points.
        d r_z = abs(x0 - x1);
        error += sqr(r_z.f);

        for (int i = 0; i < N; i++) {
          double Dr_s0_i = D(r_s0, i);
          double Dr_s1_i = D(r_s1, i);
          double Dr_z_i = D(r_z, i);
          // Add this residual to J^T*y.
          JTy(i) -= Dr_s0_i*r_s0.f;
          JTy(i) -= Dr_s1_i*r_s1.f;
          JTy(i) -= Dr_z_i*r_z.f;
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
      lambda = lambda_init * (1.0f + randf());
      prev_error = error;
      cam0 = prev_cam0;
      cam1 = prev_cam1;
      spheres = prev_spheres;
      continue;
    }

    // J^T*J <- J^J*J + lambda*diag(J^J*J)
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
            
    if (dot(dB, dB) < epsilon*epsilon) {
      log << "  converged on it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
      cam0f = camera_cast<float>(cam0);
      cam1f = camera_cast<float>(cam1);
      //for (size_t i = 0; i < sphere_observations.size(); i++)
      //  sphere_observations[i].center = vector_cast<float>(spheres[i]);
      return scalar_cast<float>(error);
    }
  }
  throw runtime_error("calibration optimization failed to converge");
}

}  // namespace ev3cv