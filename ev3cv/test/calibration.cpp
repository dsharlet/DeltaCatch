#include <vision/calibration.h>
#include "test.h"

using namespace std;
using namespace ev3cv;

int main(int argc, const char **argv) {
  // Set up some cameras with a baseline on the x axis, looking down the z axis.
  cameraf cam0(
      vector2f(176.0f, 144.0f),
      vector2f(0.05f, 0.05f),
      vector2f(1.0f, 1.0f),
      1.0f,
      quaternionf::from_basis(
          vector3f(1.0f, 0.0f, 0.0f),
          vector3f(0.0f, 1.0f, 0.0f),
          vector3f(0.0f, 0.0f, 1.0f)),
      vector3f(10.0f, 0.0f, 0.0f));
  cameraf cam1 = cam0;
  cam1.x.x *= -1.0f;
  
  // Generate some observations of samples from a sphere.
  vector<sphere_observation_set> spheres;
  for (int i = 0; i < 4; i++) {
    spheres.emplace_back();
    sphere_observation_set &sphere = spheres.back();
    sphere.center = randv3f(-30.0f, 30.0f);
    sphere.radius = randf(75.0f, 100.0f);
    while (sphere.samples.size() < 16) {
      vector3f x = unit(randv3f(-1.0f, 1.0f))*sphere.radius + sphere.center;
      if (cam0.is_visible(x) && cam1.is_visible(x)) {
        sphere.samples.push_back({
            cam0.project_to_sensor(x),
            cam1.project_to_sensor(x)
        });
      }
    }
  }
  
  // Add some noise to the cameras and run the calibration.
  throw runtime_error("not implemented");

  return 0;
}