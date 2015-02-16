#include <cl/cl.h>
#include <vision/calibration.h>
#include "test.h"

using namespace std;
using namespace ev3cv;

static cl::arg<int> sphere_count(
  4,
  cl::name("sphere-count"),
  cl::desc("Number of spheres to use for generating test data."));
static cl::arg<int> sphere_sample_count(
  16,
  cl::name("sphere-sample-count"),
  cl::desc("Number of samples per sphere to generate."));

static cl::arg<float> baseline(
  20.0f,
  cl::name("baseline"),
  cl::desc("Distance between the cameras."));

static cl::arg<int> max_iterations(
  100,
  cl::name("max-iterations"),
  cl::desc("Maximum number of iterations for calibration optimization."));
static cl::arg<float> convergence_threshold(
  1e-3f,
  cl::name("convergence-threshold"),
  cl::desc("Threshold for a step to be considered converged."));
static cl::arg<float> lambda_init(
  1.0f,
  cl::name("lambda-init"));
static cl::arg<float> lambda_decay(
  0.9f,
  cl::name("lambda-decay"));

static cl::arg<int> test_count(
  1,
  cl::name("test-count"),
  cl::desc("Number of calibration scenarios to run."));
static cl::arg<float> test_distortion(
  0.0f,
  cl::name("test-distortion"),
  cl::desc("Magnitude of synthetic distortion allowed."));
static cl::arg<float> test_init(
  0.1f,
  cl::name("test-init"),
  cl::desc("How much the camera parameters are perturbed before testing optimization."));
static cl::arg<float> epsilon(
  1e-4f,
  cl::name("epsilon"),
  cl::desc("Amount of error to allow for a test to be considered successful."));

void test_rodrigues(int count) {
  for (int i = 0; i < count; i++) {
    quaternion<double> q = unit(quaternion_cast<double>(quaternionf(randf(), randv3f())));
    ASSERT_LT(abs(q - from_rodrigues(to_rodrigues(q))), 1e-6);
  }
}

void test_filter() {
  vector<stereo_observation> flat(100, {1.0f, 1.0f});
  filter_observations(flat, 2.0f);
  for (const auto &i : flat) {
    ASSERT_LT(abs(i.x0 - vector2f(1.0f)), 1e-6f);
    ASSERT_LT(abs(i.x1 - vector2f(1.0f)), 1e-6f);
  }

  const int N = 100;
  vector<stereo_observation> line(N);
  for (int i = 0; i < N; i++) {
    line[i].x0 = vector2f(i, i);
    line[i].x1 = vector2f(-i, i);
  }

  filter_observations(line, 2.0f);
  for (int i = 10; i + 10 < N; i++) {
    ASSERT_LT(abs(line[i].x0 - vector2f(i, i)), 1e-4f);
    ASSERT_LT(abs(line[i].x1 - vector2f(-i, i)), 1e-4f);
  }
}

void test_synchronize(int count) {
  for (int i = 0; i < count; i++) {
    try {
      // Generate a circle of observations.
      float r = randf(20.0f, 50.0f);
      int N = 50;// rand()%100 + 50;
      float shift = randf(-1.0f, 1.0f);
      float theta_shift = shift*(2*pi/N);
      vector<stereo_observation> obs;
      obs.reserve(N);
      for (int j = 0; j < N; j++) {
        float theta = 2*pi*j/N;
        obs.push_back({
            vector2f(cos(theta), sin(theta))*r, 
            vector2f(cos(theta + theta_shift), sin(theta + theta_shift))*r
        });
      }

      float dx = synchronize_observations(obs);
      ASSERT_LT(abs(dx - shift), 1e-3f);

      for (int i = 10; i + 10 < N; i++) {
        ASSERT_LT(abs(obs[i].x0 - obs[i].x1), 1e-2f*max(abs(obs[i].x0), abs(obs[i].x1)));
      }
    } catch(exception &ex) {
      cout << ex.what() << endl;
    }
  }
}

void test_calibrate(int count) {
  for (int n = 0; n < count; n++) {
    try {
      // Set up some cameras with a baseline on the x axis, looking down the z axis.
      cameraf cam0 = cameraf::from_lens(
          vector2f(176.0f, 144.0f),
          randv2f(-test_distortion, *test_distortion),
          vector2f(8.0f, 8.0f),
          3.0f,
          unit(quaternionf::from_basis(
            vector3f(1.0f, 0.0f, 0.0f),
            vector3f(0.0f, 1.0f, 0.0f),
            vector3f(0.0f, 0.0f, 1.0f))),
          vector3f(-baseline/2.0f, 0.0f, 0.0f));
      cameraf cam1 = cameraf::from_lens(
          vector2f(176.0f, 144.0f),
          randv2f(-test_distortion, *test_distortion),
          vector2f(6.0f, 6.0f),
          3.0f,
          unit(quaternionf::from_basis(
            vector3f(-1.0f, 0.0f, 0.0f),
            vector3f(0.0f, -1.0f, 0.0f),
            vector3f(0.0f, 0.0f, 1.0f))),
          vector3f(baseline/2.0f, 0.0f, 0.0f));
  
      // Generate some observations of samples from a sphere.
      vector<sphere_observation_set> spheres;
      for (int i = 0; i < sphere_count; i++) {
        spheres.emplace_back();
        sphere_observation_set &sphere = spheres.back();
        sphere.center = randv3f(-30.0f, 30.0f);
        sphere.radius = randf(75.0f, 100.0f);
        while (static_cast<int>(sphere.samples.size()) < sphere_sample_count) {
          vector3f x = unit(randv3f(-1.0f, 1.0f))*sphere.radius + sphere.center;
          if (cam0.is_visible(x) && cam1.is_visible(x))
            sphere.samples.push_back({cam0.project_to_sensor(x), cam1.project_to_sensor(x)});
        }
      }
      
      // Run the calibration with a realistic initial guess.
      cameraf cam0_ = cam0;
      cameraf cam1_ = cam1;
      cam0_.a *= randv2f(1.0f - test_init, 1.0f + test_init);
      cam1_.a *= randv2f(1.0f - test_init, 1.0f + test_init);
      cam0_.d1 *= randv2f(1.0f - test_init, 1.0f + test_init);
      cam1_.d1 *= randv2f(1.0f - test_init, 1.0f + test_init);
      cam0_.c *= randv2f(1.0f - test_init, 1.0f + test_init);
      cam1_.c *= randv2f(1.0f - test_init, 1.0f + test_init);
      cam0_.s *= randf(1.0f - test_init, 1.0f + test_init);
      cam1_.s *= randf(1.0f - test_init, 1.0f + test_init);
      //cam0_.R = unit(quaternionf(randf(), randv3f()));
      //cam1_.R = unit(quaternionf(randf(), randv3f()));

      calibrate(
          spheres, 
          cam0_, cam1_, 
          cout, "d1aatR", 
          max_iterations, convergence_threshold, 
          lambda_init, lambda_decay);
  
      ASSERT_LT(abs(cam0_.d1 - cam0.d1), epsilon);
      ASSERT_LT(abs(cam0_.a - cam0.a), epsilon);
      ASSERT_LT(abs(cam0_.s - cam0.s), epsilon);
      ASSERT_LT(abs(cam0_.c - cam0.c), epsilon);
      ASSERT_LT(abs(cam0_.R - cam0.R), epsilon);
      ASSERT_LT(abs(cam0_.x - cam0.x), epsilon);

      ASSERT_LT(abs(cam1_.d1 - cam1.d1), epsilon);
      ASSERT_LT(abs(cam1_.a - cam1.a), epsilon);
      ASSERT_LT(abs(cam1_.s - cam1.s), epsilon);
      ASSERT_LT(abs(cam1_.c - cam1.c), epsilon);
      ASSERT_LT(abs(cam1_.R - cam1.R), epsilon);
      ASSERT_LT(abs(cam1_.x - cam1.x), epsilon);
    } catch (exception &ex) {
      cout << ex.what() << endl;
    }
  }
}

int main(int argc, const char **argv) {
  cl::parse(argv[0], argc - 1, argv + 1);

  test_rodrigues(1000);
  test_synchronize(10);
  test_filter();
  test_calibrate(test_count);

  return 0;
}