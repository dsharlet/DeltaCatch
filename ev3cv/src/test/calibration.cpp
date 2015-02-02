#include <vision/calibration.h>
#include "test.h"

using namespace std;
using namespace ev3cv;

const int sphere_count = 4;
const int sample_count = 16;
const float baseline = 20.0f;
const float epsilon = 5e-3f;

int main(int argc, const char **argv) {
  // Set up some cameras with a baseline on the x axis, looking down the z axis.
  cameraf cam0(
      vector2f(176.0f, 144.0f),
      vector2f(0.0f, 0.0f),
      vector2f(1.0f, 1.0f),
      2.0f,
      quaternionf::from_basis(
          vector3f(1.0f, 0.0f, 0.0f),
          vector3f(0.0f, 1.0f, 0.0f),
          vector3f(0.0f, 0.0f, 1.0f)),
      vector3f(baseline/2.0f, 0.0f, 0.0f));
  cameraf cam1 = cam0;
  cam1.x.x *= -1.0f;
  
  // Generate some observations of samples from a sphere.
  vector<sphere_observation_set> spheres;
  for (int i = 0; i < sphere_count; i++) {
    spheres.emplace_back();
    sphere_observation_set &sphere = spheres.back();
    sphere.center = randv3f(-30.0f, 30.0f);
    sphere.radius = randf(75.0f, 100.0f);
    while (sphere.samples.size() < sample_count) {
      vector3f x = unit(randv3f(-1.0f, 1.0f))*sphere.radius + sphere.center;
      if (cam0.is_visible(x) && cam1.is_visible(x)) {
        sphere.samples.push_back({
            cam0.project_to_sensor(x),
            cam1.project_to_sensor(x)
        });
      }
    }
  }

  // Run the calibration with a realistic initial guess.
  cameraf cam0_ = cam0;
  cameraf cam1_ = cam1;

  cam0_.d1 = vector2f(0.0f);
  cam1_.d1 = vector2f(0.0f);
  cam0_.a.x *= randf(0.9f, 1.1f);
  cam0_.a.y *= randf(0.9f, 1.1f);
  cam1_.a.x *= randf(0.9f, 1.1f);
  cam1_.a.y *= randf(0.9f, 1.1f);
  cam0_.s = 0.0f;
  cam1_.s = 0.0f;
  cam0_.t = vector2f(0.0f);
  cam1_.t = vector2f(0.0f);
  // TODO: Figure out how to perturb R reasonably.

  calibrate(spheres, cam0_, cam1_, cout);
  
  ASSERT_LT(abs(cam0_.d1 - cam0.d1), epsilon);
  ASSERT_LT(abs(cam0_.a - cam0.a), epsilon);
  ASSERT_LT(abs(cam0_.s - cam0.s), epsilon);
  ASSERT_LT(abs(cam0_.t - cam0.t), epsilon);
  ASSERT_LT(abs(cam0_.R - cam0.R), epsilon);

  ASSERT_LT(abs(cam1_.d1 - cam1.d1), epsilon);
  ASSERT_LT(abs(cam1_.a - cam1.a), epsilon);
  ASSERT_LT(abs(cam1_.s - cam1.s), epsilon);
  ASSERT_LT(abs(cam1_.t - cam1.t), epsilon);
  ASSERT_LT(abs(cam1_.R - cam1.R), epsilon);

  return 0;
}