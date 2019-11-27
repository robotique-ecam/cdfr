#include <cstdio>
#include <string>
#include <fstream>
#include <iostream>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

class I2C {
public:
  I2C(int i2c_bus) {
    filename_ = "/dev/i2c-" + std::to_string(i2c_bus);
    i2c_fd_ = open(filename_, O_RDWR);
  }

  int set_address(int addr) {
    ioctl(i2c_fd_, I2C_SLAVE, cmd);
  }

  int read_byte(uint8_t cmd) {
    return i2c_smbus_read_byte(i2c_fd_);
  }

  uint16_t read_word(uint8_t cmd) {
    return i2c_smbus_read_word_data(i2c_fd_, cmd);
  }

  void write_byte(uint8_t cmd) {
    i2c_smbus_write_byte(i2c_fd_, cmd);
  }

private:
  static int i2c_fd_;
  std::string filename_;

};
