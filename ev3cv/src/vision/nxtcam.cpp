#include <vision/nxtcam.h>

#include <string.h>
#include <thread>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std;

namespace ev3cv {

nxtcam::nxtcam(const std::string &port, int address) : fd_(-1), port_(port) {
  // Find the port number from the port string.
  if (port[0] != 'i' && port[1] != 'n')
    throw runtime_error("port is not an input port.");
  char path[32];
  sprintf(path, "/dev/i2c-%d", port[2] - '0' + 2);

  fd_ = open(path, O_RDWR);
  if (fd_ < 0)
    throw runtime_error(strerror(errno));

  if (ioctl(fd_, I2C_SLAVE, address) < 0)
    throw runtime_error(strerror(errno));
}

nxtcam::~nxtcam() {
  if (fd_ >= 0)
    close(fd_);
}
  
string nxtcam::version() const {
  // This is reading one extra register for each field... probably harmless.
  char ret[9];
  read(reg_version, ret);
  ret[8] = 0;
  return ret;
}
  
string nxtcam::vendor_id() const {
  // This is reading one extra register for each field... probably harmless.
  char ret[9];
  read(reg_vendor_id, ret);
  ret[8] = 0;
  return ret;
}
  
string nxtcam::device_id() const {
  // This is reading one extra register for each field... probably harmless.
  char ret[9];
  read(reg_device_id, ret);
  ret[8] = 0;
  return ret;
}

// Command camera to begin tracking objects.
void nxtcam::track_objects() {
  char cmds[] = "DABE";
  write_cmds(cmds);
}

// Command camera to begin tracking lines.
void nxtcam::track_lines() {
  char cmds[] = "DXLE";
  write_cmds(cmds);
}

void nxtcam::stop_tracking() {
  char cmds[] = "D";
  write_cmds(cmds);
}
  
// Update the tracking data.
nxtcam::blob_list nxtcam::blobs() const {
  uint8_t blob_count[1];
  read(reg_count, blob_count);
  if (blob_count[0] > 8)
    throw runtime_error("unsupported blob count.");
  blob_list blobs(blob_count[0]);

  if (!blobs.empty()) {
    vector<uint8_t> data(blobs.size()*5);
    read(reg_data, &data[0], data.size());
    for (size_t i = 0; i < blobs.size(); i++) {
      blobs[i].color = data[i*5 + 0];
      blobs[i].x1 = vector2i(data[i*5 + 1], data[i*5 + 2]);
      blobs[i].x2 = vector2i(data[i*5 + 3], data[i*5 + 4]);
    }
  }

  return std::move(blobs);
}

// Send an ordered sequence of commands to the device.
void nxtcam::write(uint8_t reg, uint8_t data) {
  uint8_t buf[2] = { reg, data };
  if (::write(fd_, buf, sizeof(buf)) < 0)
    throw runtime_error(strerror(errno));

  // TODO: This really seems to be necessary, unfortunately.
  this_thread::sleep_for(chrono::milliseconds(200));
}
  
void nxtcam::read(uint8_t reg, uint8_t *data, size_t size) const {
  uint8_t addr[1] = { reg };
  if (::write(fd_, addr, sizeof(addr)) < 0)
    throw runtime_error(strerror(errno));
  if (::read(fd_, data, size) < 0)
    throw runtime_error(strerror(errno));
}

}  // namespace ev3cv