#include <random>
#include <chrono>

#include <cl/cl.h>

#include "debug.h"
#include "trajectory.h"

using namespace std;

static cl::group optimization_group("Optimization parameters");
static cl::arg<int> max_iterations(
  32,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations allowed when solving optimization problems."),
  optimization_group);
static cl::arg<float> epsilon(
  1e-3f,
  cl::name("epsilon"),
  cl::desc("Number to consider to be zero when solving optimization problems."),
  optimization_group);
static cl::arg<float> lambda_recovery(
  1.0f,
  cl::name("lambda-init"),
  cl::desc("Initial value of Levenberg-Marquardt damping parameter."),
  optimization_group);
static cl::arg<float> lambda_decay(
  0.1f,
  cl::name("lambda-decay"),
  cl::desc("Decay ratio of the Levenberg-Marquardt damping parameter on a successful iteration."),
  optimization_group);

// A few helper print functions for debug output from this file.
std::ostream &operator <<(std::ostream &os, const observation &obs) {
  return os << obs.t << " (" << obs.f << ")";
}

// Defines the objective function for a trajectory/ellipse intersection.
template <typename T>
T trajectory_sphere(float half_g, const trajectoryf &tj, const vector3f &s, float sqr_r, const T &t) {
  vector3<T> x = tj.position_half_g<T>(half_g, t);
  return sqr_abs(x - s) - sqr_r;
}

// Use Newton's method to find an intersection of a trajectory and an ellipse, using t_0 as the initial guess.
float intersect_trajectory_sphere(float half_g, const trajectoryf &tj, const vector3f &s, float sqr_r, float t) {
  typedef diff<float, 1> d;
  d ft;
  for (int i = 0; i < max_iterations; i++) {
    ft = trajectory_sphere<d>(half_g, tj, s, sqr_r, d(t, 0));
    t -= ft.u/D(ft);

    if (abs(ft.u) < epsilon)
      return t;
  }
  throw runtime_error("no trajectory-ellipse intersection found");
}

// This function finds the first intersection after t of a trajectory and a ellipse.
float intersect_trajectory_sphere(float g, const trajectoryf &tj, const vector3f &s, float r, float t_min, float t_max) {
  float sqr_r = sqr(r);
  float dt = (t_max - t_min)/20.0f;
  for (float t0 = t_min; t0 < t_max; t0 += dt) {
    float t = intersect_trajectory_sphere(g/2.0f, tj, s, sqr_r, t0);
    if (t_min < t && t < t_max)
      return t;
  }
  throw runtime_error("no trajectory-ellipse intersection found");
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
    x = tj.position_half_g(half_g, T(ob.t));
  }
  vector2<T> r = cam.project_to_focal_plane(x) - ob.f;
  return -r;
}

// Estimates a trajectory t given a set of observations from two cameras. The input
// value of tj is used as an initial guess for optimization. 
float estimate_trajectory(
    float gravity, 
    const cameraf &cam0, const cameraf &cam1,
    observation_buffer &obs0, observation_buffer &obs1,
    float &dtf,
    trajectoryf &tjf) {
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

  if (obs0.size() + obs1.size() < N)
    throw runtime_error("not enough observations");

  d dt = d(dtf, t);
  trajectory<d> tj;
  tj.x = vector3<d>(d(tjf.x.x, x_x), d(tjf.x.y, x_y), d(tjf.x.z, x_z));
  tj.v = vector3<d>(d(tjf.v.x, v_x), d(tjf.v.y, v_y), d(tjf.v.z, v_z));
  
  size_t M0 = static_cast<int>(obs0.size());
  size_t M1 = static_cast<int>(obs1.size());
  size_t M = M0 + M1;
  
  dbg(1) << "estimate_trajectory, M=" << M << " (" << M0 << " + " << M1 << ")..." << endl;

  // Levenberg-Marquardt state.
  trajectory<d> prev_tj = tj;
  float prev_error = std::numeric_limits<float>::infinity();
  float lambda = lambda_recovery;

  int it;
  for (it = 1; it <= max_iterations; it++) {
    // Compute J^T*J and b.
    matrix<float, N, N> JTJ;
    matrix<float, N, 1> JTy;
    float error = 0.0f;
    for (size_t i = 0; i < M; i++) {
      const observation &o_i = i < M0 ? obs0[obs0.begin() + i] : obs1[obs1.begin() + i - M0];

      vector2<d> r = reprojection_error(
          half_g, 
          i < M0 ? cam0 : cam1, 
          o_i, 
          i < M0 ? nullptr : &dt, tj);

      error += sqr(r.x.u) + sqr(r.y.u);

      for (int i = 0; i < N; i++) {
        float Dr_x_i = D(r.x, i);
        float Dr_y_i = D(r.y, i);
        // Add this residual to J^T*y.
        JTy(i) -= Dr_x_i*r.x.u + Dr_y_i*r.y.u;
        // Add this residual to J^T*J
        for (int j = 0; j < N; j++)
          JTJ(i, j) += Dr_x_i*D(r.x, j) + Dr_y_i*D(r.y, j);
      }
    }

    // If error increased, throw away the previous iteration and 
    // reset the Levenberg-Marquardt damping parameter.
    if (error > prev_error) {
      dbg(2) << "  it=" << it << ", ||dB||=<bad iteration>, error=" 
        << error << ", lambda=" << lambda << endl;
      lambda = lambda_recovery;
      prev_error = error;
      tj = prev_tj;
      continue;
    }

    // J^T*J <- J^J*J + lambda*diag(J^J*J)
    for (int i = 0; i < N; i++)
      JTJ(i, i) *= 1.0f + lambda;

    // Solve J^T*J*dB = J^T*y.
    matrix_ref<float, N, 1> dB = solve(JTJ, JTy);
    
    if (!isfinite(dB))
      throw runtime_error("estimate_trajectory optimization diverged");

    // If the debug level is high, dump out info about the last few iterations.
    if (dbg_level() >= 4 && it + dbg_level() >= max_iterations) {
      dbg(4) << "  it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) 
          << ", error=" << error << ", lambda=" << lambda << endl;
    }

    // Update Levenberg-Marquardt damping parameter.
    lambda *= lambda_decay;
    prev_error = error;
    prev_tj = tj;

    tj.x += vector3f(dB(x_x), dB(x_y), dB(x_z));
    tj.v += vector3f(dB(v_x), dB(v_y), dB(v_z));
    dt += dB(t);

    if (dot(dB, dB) < epsilon_sq) {
      if (dbg_level() >= 3) {
        dbg(3) << "  converged on it=" << it << ", ||dB||=" << sqrt(dot(dB, dB)) << endl;
      }
      break;
    }
  }

  tjf.x = vector_cast<float>(tj.x);
  tjf.v = vector_cast<float>(tj.v);
  dtf = scalar_cast<float>(dt);

  dbg(2) << "estimate_trajectory finished, it=" << it << endl;

  return 0.0f;
}
