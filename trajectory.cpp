#include <random>
#include <chrono>

#include <cl.h>

#include "debug.h"
#include "matrix.h"
#include "autodiff.h"
#include "trajectory.h"

using namespace std;

static cl::arg<int> max_iterations(
  12,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."));
static cl::arg<float> epsilon(
  1e-3,
  cl::name("epsilon"),
  cl::desc("Number to consider to be zero when solving optimization problems."));

// A few helper print functions for debug output from this file.
std::ostream &operator <<(std::ostream &os, const observation &obs) {
  return os << obs.t << " (" << obs.f << ")";
}

template <typename T, int N>
std::ostream &operator << (std::ostream &os, const diff<T, N> &d) {
  return os << d.x;
}

// Defines the objective function for a trajectory/sphere intersection.
template <typename T>
T trajectory_sphere_XZ(float half_g, float v_x, float v_z, const vector3f &s, float r, const T &t) {
  return sqr(v_x*t - s.x) + sqr(half_g*sqr(t) + v_z*t - s.z) + sqr(s.y) - sqr(r);
}

// Use Newton's method to find an intersection of a trajectory in the XZ plane, and a sphere, using t_0 as the initial guess.
float intersect_trajectory_sphere_XZ(float half_g, float v_x, float v_z, const vector3f &s, float r, float t_0) {
  typedef diff<float, 1> d;
  float t = t_0;
  d ft;
  for (int i = 0; i < max_iterations; i++) {
    ft = trajectory_sphere_XZ(half_g, v_x, v_z, s, r, d(t, 0));
    t -= ft.f/D(ft, 0);

    if (abs(ft.f) < epsilon)
      break;
  }
  return t;
}

// This function finds the first intersection after t of a trajectory and a sphere.
float intersect_trajectory_sphere(float g, const trajectoryf &tj, const pair<vector3f, float> &s, float t_min, float t_max) {
  const float half_g = g/2.0f;

  // Project the sphere onto the plane containing the trajectory.
  vector3f X = vector3f(tj.v.x, tj.v.y, 0.0f);
  float v_x = abs(X);
  X /= v_x;
  vector3f Z = vector3f(0.0f, 0.0f, 1.0f);
  vector3f Y = cross(X, Z);

  // This basis contains the trajectory in the XZ plane.
  basis3f B_tj(X, Y, Z, tj.x);

  // Map the sphere into the basis given by the trajectory.
  vector3f s0 = B_tj.to_local(s.first);

  // Now we need to solve this:
  // 
  //   (v_x*t - s0.x)^2 + (g*t^2 + tj.v.z*t - s0.z)^2 + s0.y^2 = r^2
  //
  // It's a quartic. Rather than use the quartic formula (pain in the ass), let's use Newton's method.
  return intersect_trajectory_sphere_XZ(g/2.0f, v_x, tj.v.z, s0, s.second, (t_min + t_max)/2.0f);
}

// Find the intersection of a trajectory with the z plane. This function computes the 
// later (larger t) of the two intersections.
float intersect_trajectory_zplane(float g, const trajectoryf &tj, float z) {
  float a = g/2.0f;
  float b = tj.v.z;
  float c = tj.x.z - z;
  float D = sqr(b) - 4.0f*a*c;
  if (D < 0)
    throw runtime_error("trajectory has no intercept with z plane");

  return (-b - sqrt(D))/(2.0f*a);
}

typedef circular_array<observation, 128> observation_buffer;

// Evaluates the cost function to be optimized.
template <typename T>
vector2<T> reprojection_error(
    float half_g,
    const cameraf &cam, 
    const observation &ob,
    const T* dt, 
    const trajectory<T> &tj) {
  // Compute the error in screen space for the observation.
  vector3<T> x;
  if (dt) {
    x = tj.position_half_g(half_g, *dt + ob.t);
  } else {
    x = tj.position_half_g(half_g, ob.t);
  }
  vector2<T> r = cam.project_to_focal_plane(x) - ob.f;
  return -r;
}

// Estimates a trajectory t given a set of observations from two cameras. The input
// value of tj is used as an initial guess for optimization. 
int estimate_trajectory(
    float gravity, 
    float sigma_observation, 
    float outlier_threshold, 
    const cameraf &cam0, const cameraf &cam1,
    observation_buffer &obs0, observation_buffer &obs1,
    float &dt,
    trajectoryf &tj) {
  const float epsilon_sq = epsilon*epsilon;
  const float half_g = gravity/2;

  enum variable {
    t = 0,
    x_x, x_y, x_z,
    v_x, v_y, v_z,
    // The number of variables.
    N,
  };
  typedef diff<float, N> d;
  
  d dt_ = d(dt, t);
  trajectory<d> tj_;
  tj_.x = vector3<d>(d(tj.x.x, x_x), d(tj.x.y, x_y), d(tj.x.z, x_z));
  tj_.v = vector3<d>(d(tj.v.x, v_x), d(tj.v.y, v_y), d(tj.v.z, v_z));
  
  int M0 = static_cast<int>(obs0.size());
  int M1 = static_cast<int>(obs1.size());
  int M = M0 + M1;
  
  dbg(3) << "estimate_trajectory, M=" << M << "..." << endl;
  
  int it;
  for (it = 1; it <= max_iterations; it++) {
    // Compute J^T*J and b.
    matrix<float, N, N> JTJ;
    matrix<float, N, 1> JTy;
    for (size_t i = 0; i < M; i++) {
      const observation &o_i = i < M0 ? obs0[obs0.begin() + i] : obs1[obs1.begin() + i - M0];
      if (o_i.outlier) continue;

      vector2<d> r = reprojection_error(
          half_g, 
          i < M0 ? cam0 : cam1, 
          o_i, 
          i < M0 ? nullptr : &dt_, tj_);

      for (int i = 0; i < N; i++) {
        float Dr_x_i = D(r.x, i);
        float Dr_y_i = D(r.y, i);
        // Add this residual to J^T*y.
        JTy(i) -= Dr_x_i*r.x.f + Dr_y_i*r.y.f;
        // Add this residual to J^T*J
        for (int j = 0; j < N; j++)
          JTJ(i, j) += Dr_x_i*D(r.x, j) + Dr_y_i*D(r.y, j);
      }
    }
    
    // Solve J^T*J*dB = J^T*y.
    matrix_ref<float, N, 1> dB = solve(JTJ, JTy);
    
    if (!isfinite(dB))
      throw runtime_error("estimate_trajectory optimization diverged");

    // If the debug level is high, dump out info about the last few iterations.
    if (dbg_level() >= 4 && it + dbg_level() >= max_iterations) {
      dbg(4) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
    }

    tj_.x += vector3f(dB(x_x), dB(x_y), dB(x_z));
    tj_.v += vector3f(dB(v_x), dB(v_y), dB(v_z));
    dt_ += dB(t);

    if (dot(dB, dB) < epsilon_sq) {
      if (dbg_level() >= 3)
        dbg(3) << "  converged on it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
      break;
    }
  }

  tj.x = vector_cast<float>(tj_.x);
  tj.v = vector_cast<float>(tj_.v);
  dt = scalar_cast<float>(dt_);

  dbg(3) << "  tagging outliers..." << endl;
  int outliers = 0;
  const float error_threshold = sigma_observation*outlier_threshold;
  for (size_t i = 0; i < M; i++) {
    observation &o_i = i < M0 ? obs0[obs0.begin() + i] : obs1[obs1.begin() + i - M0];
    if (o_i.outlier) continue;

    vector2<float> r = reprojection_error(
        half_g, 
        i < M0 ? cam0 : cam1, 
        o_i, 
        i < M0 ? nullptr : &dt, tj);

    if (abs(r.x) > error_threshold || abs(r.y) > error_threshold) {
      if (dbg_level() >= 4)
        dbg(4) << "    outlier found, std error = " << vector2f(abs(r.x)/sigma_observation, abs(r.y)/sigma_observation) << endl;
      o_i.outlier = true;
      outliers++;
    }
  }
  dbg(3) << "  " << outliers << " outliers tagged." << endl;

  dbg(2) << "estimate_trajectory finished, it=" << it << ", outliers=" << outliers << endl;

  return outliers;
}

static cl::group test_group("Trajectory estimation test parameters");

static cl::arg<int> test_count(
  10,
  cl::name("test-trajectory-count"),
  cl::desc("Number of trajectories to test estimate_trajectory on."),
  test_group);
static cl::arg<float> flight_time(
  1.25f,
  cl::name("test-flight-time"),
  cl::desc("Flight time of test trajectories, in seconds."),
  test_group);
static cl::arg<float> target_distance(
  250.0f,
  cl::name("test-distance"),
  cl::desc("How far away from the target the simulated trajectories originate, in studs."),
  test_group);
static cl::arg<float> false_negative_rate(
  0.1f,
  cl::name("test-false-negative-rate"),
  cl::desc("Probability of missed observations in simulated trajectory observations."),
  test_group);
static cl::arg<float> false_positive_rate(
  0.0f,
  cl::name("test-false-positive-rate"),
  cl::desc("Probability of spurious observations in simulated trajectory observations."),
  test_group);
static cl::arg<float> tolerance(
  4.0f,
  cl::name("test-tolerance"),
  cl::desc("Allowed error in estimated intercept for a test to be considered successful, in studs."),
  test_group);

// Test and benchmark estimate_trajectory.
void test_estimate_trajectory(
    float gravity, 
    float sigma_observation, 
    float outlier_threshold, 
    const cameraf &cam0, const cameraf &cam1) {
  // How many random trajectories to check.
  const int count = test_count;
  // Sampling rate of the generated observations.
  const float T = 1.0f/30.0f;
  
  default_random_engine rnd;
  normal_distribution<float> obs_noise(0.0f, sigma_observation);

  // Trajectory gemoetry.
  const pair<vector3f, float> launch = { { 0.0f, target_distance, 0.0f }, 100.0f };
  const pair<vector3f, float> target = { { 0.0f, 0.0f, 12.0f }, 20.0f };

  trajectoryf tj_init;
  tj_init.x = launch.first;
  tj_init.v = target.first - launch.first;
  tj_init.v /= flight_time;
  tj_init.v.z += -0.5f*gravity*flight_time;

  dbg(1) 
    << "Test trajectory max z=" << tj_init.position(gravity, flight_time/2).z 
    << ", target=" << tj_init.position(gravity, intersect_trajectory_zplane(gravity, tj_init, target.first.z)) << endl;
  
  // Benchmarking timer duration.
  typedef chrono::high_resolution_clock clock;
  clock::duration benchmark = clock::duration::zero();
  int benchmark_count = 0;

  int fails = 0;
  float total_err = 0.0;

  for (int i = 0; i < count; i++) {
    // Use a random time shift of +/- 1 frame.
    float dt = (randf()*2.0f - 1.0f)*T;

    // Generate a trajectory to test with.
    trajectoryf tj = tj_init;
    tj.x = unit(randv3f(-1.0f, 1.0f))*launch.second + launch.first;
    tj.v = unit(randv3f(-1.0f, 1.0f))*target.second + target.first - tj.x;
    tj.v /= flight_time;
    tj.v.z += -0.5f*gravity*flight_time;

    // Generate some simulated observations of the trajectory, 
    // adding some random noise/false positives/false negatives.
    observation_buffer obs0, obs1;
    for (float t = 0.0f; t <= flight_time; t += T) {
      if (randf() >= false_negative_rate) {
        vector3f x = tj.position(gravity, t);
        vector2f ob = cam0.project_to_normalized(x) + vector2f(obs_noise(rnd), obs_noise(rnd));
        if (-1.0f <= ob.x && ob.x <= 1.0f && 
            -1.0f <= ob.y && ob.y <= 1.0f && 
            cam0.transform.to_local(x).z > tolerance)
          obs0.push_back({t, cam0.normalized_to_focal_plane(ob), false});
      }
      if (randf() >= false_negative_rate) {
        vector3f x = tj.position(gravity, t + dt);
        vector2f ob = cam1.project_to_normalized(x) + vector2f(obs_noise(rnd), obs_noise(rnd));
        if (-1.0f <= ob.x && ob.x <= 1.0f && 
            -1.0f <= ob.y && ob.y <= 1.0f && 
            cam1.transform.to_local(x).z > tolerance)
          obs1.push_back({t, cam1.normalized_to_focal_plane(ob), false});
      }
      while (randf() < false_positive_rate)
        obs0.push_back({t, cam0.normalized_to_focal_plane(randv2f(-1.0f, 1.0f))});
      while (randf() < false_positive_rate)
        obs1.push_back({t, cam1.normalized_to_focal_plane(randv2f(-1.0f, 1.0f))});
    }
    
    try {
      // Estimate the trajectory from the random observations.
      float dt_ = 0.0f;
      trajectoryf tj_ = tj_init;
      auto start = clock::now();
      int outliers = estimate_trajectory(
          gravity, 
          sigma_observation, outlier_threshold, 
          cam0, cam1,
          obs0, obs1, 
          dt_, tj_);
      auto finish = clock::now();
      benchmark_count++;

      // Check that the trajectory is within tolerance.
      vector3f intercept_tj = tj.position(gravity, intersect_trajectory_zplane(gravity, tj, target.first.z));
      vector3f intercept_tj_ = tj_.position(gravity, intersect_trajectory_zplane(gravity, tj_, target.first.z));
      vector3f err = intercept_tj - intercept_tj_;
      total_err += abs(err);
      size_t M = obs0.size() + obs1.size();
      if (abs(err) > tolerance || isnan(err)) {
        cerr << "Target intercept test failed, ||actual-estimated||=" << abs(err) << ", M=" << M << ", outliers=" << outliers << endl;
        cerr << "  Actual intercept=" << intercept_tj << endl;
        cerr << "  Estimated intercept=" << intercept_tj_ << endl;
        cerr << "  Actual trajectory: x=" << tj.x << ", v=" << tj.v << ", dt=" << dt << endl;
        cerr << "  Estimated trajectory: x=" << tj_.x << ", v=" << tj_.v << ", dt=" << dt_ << endl;
        fails++;
      } else {
        dbg(1) << "Target intercept success, ||actual-estimated||=" << abs(err) << ", M=" << M << ", outliers=" << outliers << endl;
        dbg(4) << "  Actual intercept=" << intercept_tj << endl;
        dbg(4) << "  Estimated intercept=" << intercept_tj_ << endl;
        dbg(2) << "  Actual trajectory: x=" << tj.x << ", v=" << tj.v << ", dt=" << dt << endl;
        dbg(2) << "  Estimated trajectory: x=" << tj_.x << ", v=" << tj_.v << ", dt=" << dt_ << endl;
        // Only include benchmark time if the optimization was successful.
        benchmark += finish - start;
      }
    } catch(exception &ex) {
      cerr << "Test fail: " << ex.what() << endl;
      fails++;
    }
  }

  if (fails > 0)
    cerr << fails << " tests failed!" << endl;

  dbg(1) << "estimate_trajectory accuracy=" << total_err/count << ", benchmark=" << 1e3f*static_cast<float>(benchmark.count())/benchmark_count/clock::period::den << " ms" << endl;
}