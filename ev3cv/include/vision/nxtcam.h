#ifndef EV3CV_VISION_NXTCAM_H
#define EV3CV_VISION_NXTCAM_H

#include <vector>
#include <string>

#include "../ev3cv.h"

namespace ev3cv {
  
// Given a ev3dev port specification, return a path to the corresponding I2C device.
std::string port_to_i2c_path(const std::string &port);

// Talks to an NXTcam device to perform image based tracking of 'blobs'.
class nxtcam {
public:
  // Description of a tracked blob.
  struct blob {
    vector2i x1, x2;
    int color;

    vector2f center() const { return vector_cast<float>(x1 + x2)/2.0f; }
  };
  // Type of a list of blobs.
  typedef std::vector<blob> blob_list;

  nxtcam(const std::string &path, int address = 0x01);
  ~nxtcam();
  
  // Query information about the connected device.
  std::string version() const;
  std::string vendor_id() const;
  std::string device_id() const;

  // Command camera to begin tracking objects.
  void track_objects();
  // Command camera to begin tracking lines.
  void track_lines();
  // Command camera to stop tracking.
  void stop_tracking();
  
  // Get the currently tracked blobs from the camera.
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