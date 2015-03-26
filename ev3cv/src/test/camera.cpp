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

#include <ev3cv.h>
#include <vision/camera.h>
#include "test.h"

using namespace std;
using namespace ev3cv;

int main(int argc, const char **argv) {
  for (int i = 0; i < 50; i++) {
    // Generate a (reasonable) random camera.
    camera<double> cam;
    cam.resolution = vector_cast<double>(randv2f(1.0f, 1000.0f));
    cam.d1 = vector_cast<double>(randv2f(-1e-1, 1e-1));

    cam.a = vector_cast<double>(randv2f(1e-6f, 1.0f));
    cam.s = randf(-1.0f, 1.0f);
    cam.c = vector_cast<double>(randv2f(-10.0f, 10.0f));

    cam.R = quaternion_cast<double>(unit(quaternionf(randf(), randv3f())));
    cam.x = vector_cast<double>(randv3f(-10.0f, 10.0f));

    for (int j = 0; j < 100; j++) {
      // Get a random sensor psoition to test with.
      vector2d s = vector_cast<double>(randv2f(0.0f, vector_cast<float>(cam.resolution)));
      vector2d f = cam.sensor_to_focal_plane(s);

      ASSERT_LT(abs(s - cam.focal_plane_to_sensor(f)), 1);
      ASSERT_LT(abs(s - cam.project_to_sensor(cam.sensor_to_projection(s, 1.0))), 1);
      ASSERT_LT(abs(f - cam.project_to_focal_plane(cam.focal_plane_to_projection(f, 1.0))), 1e-6);
    }
  }
  return 0;
}
