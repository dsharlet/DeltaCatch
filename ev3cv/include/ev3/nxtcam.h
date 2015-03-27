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

/** \file nxtcam.h
 * Support for NXTcam in ev3dev.
 */

#ifndef EV3CV_EV3_NXTCAM_H
#define EV3CV_EV3_NXTCAM_H

#include <vector>
#include <string>

#include "../ev3cv.h"

namespace ev3cv {

/** Given a ev3dev port specification, return a path to the corresponding I2C device.
 * For example, 'in1' maps to '/dev/i2c-3'. */
std::string port_to_i2c_path(const std::string &port);

/** Provides basic NXTcam support in ev3dev. */
class nxtcam {
public:
  /** Description of a tracked blob. */
  struct blob {
    /** Corners of the bounding box of the blob. */
    vector2i x1, x2;

    /** Index of the color of the detected blob. */
    int color;

    /** Compute the center of the bounding box of the blob. */
    vector2f center() const { return vector_cast<float>(x1 + x2)/2.0f; }
  };

  typedef std::vector<blob> blob_list;

  /** Connect to an NXTcam at the given path. To convert an ev3dev input port
   * specification to a path, see ev3cv::port_to_i2c_path. */
  nxtcam(const std::string &path, int address = 0x01);
  ~nxtcam();

  /** Query information about the connected device. */
  ///@{
  std::string version() const;
  std::string vendor_id() const;
  std::string device_id() const;
  ///@}

  /** Begin tracking objects. */
  void track_objects();
  /** Begin tracking lines. */
  void track_lines();
  /** Stop tracking. */
  void stop_tracking();

  /** Get the currently detected blobs from the camera. */
  blob_list blobs() const;

protected:
  // NXTcam registers.
  enum cam_reg {
    reg_version = 0x00,
    reg_vendor_id = 0x08,
    reg_device_id = 0x10,
    reg_cmd = 0x41,
    reg_count = 0x42,
    reg_data = 0x43,
  };

  // Read size registers beginning at reg.
  void read(uint8_t reg, uint8_t *data, size_t size) const;
  // Write a single byte to a reg.
  void write(uint8_t reg, uint8_t data);

  // Send an ordered sequence of commands to the device.
  template <typename T, int N>
  void write_cmds(T (&commands)[N]) {
    for (int i = 0; i < N; i++) {
      write(reg_cmd, commands[i]);
    }
  }

  // Read N registers beginning at reg.
  template <typename T, int N>
  void read(uint8_t reg, T (&data)[N]) const {
    read(reg, reinterpret_cast<uint8_t*>(&data[0]), N*sizeof(T));
  }

  int fd_;
};

}  // namespace ev3cv

#endif
