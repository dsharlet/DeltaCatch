#include <iostream>
#include <cassert>
#include <cmath>

using namespace std;

#include "camera.h"
#include "test.h"

int main(int argc, const char **argv) {
  for (int i = 0; i < 10; i++) {
    cameraf cam;
    cam.aspect_ratio = randf() + 0.5f;
    cam.distortion = randv2f(-1.0f, 1.0f)*1e-1f;
    cam.focal_length = randf()*10.0f + 1.0f;
    cam.resolution = randv2f(100.0f, 1000.0f);
    cam.size = randf()*10.0f + 1.0f;
  
    cam.focal_plane.x = unit(randv2f());
    cam.focal_plane.y = vector2f(-cam.focal_plane.x.y, cam.focal_plane.x.x);
    cam.focal_plane.origin = randv2f(-5.0f, 5.0f);

    // Use a random unit quaternion to generate an orthonormal basis.
    quaternionf q(randf(), randv3f());
    q /= abs(q);
    cam.transform = basis3f(q, randv3f(-5.0f, 5.0f));
  
    for (int j = 0; j < 100; j++) {
      // Get a random sensor psoition to test with.
      vector2f s(randf(), randf());
      s *= cam.resolution;
      vector2f n = cam.sensor_to_normalized(s);
      vector2f f = cam.sensor_to_focal_plane(s);

      ASSERT_LT(abs(s - cam.normalized_to_sensor(cam.sensor_to_normalized(s))), 1.0f);
      ASSERT_LT(abs(n - cam.focal_plane_to_normalized(cam.normalized_to_focal_plane(n))), 1e-6f);
      ASSERT_LT(abs(f - cam.project_to_focal_plane(cam.focal_plane_to_projection(f, 1.0f))), 1e-6f);

      ASSERT_LT(abs(s - cam.focal_plane_to_sensor(cam.sensor_to_focal_plane(s))), 1.0f);
      ASSERT_LT(abs(s - cam.project_to_sensor(cam.sensor_to_projection(s, 1.0f))), 1.0f);

      ASSERT_LT(abs(s - cam.project_to_normalized(cam.normalized_to_projection(s, 1.0f))), 1e-6f);
    }
  }  
  return 0;
}