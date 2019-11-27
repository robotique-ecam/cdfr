#include <cstdio>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>


class I2C {
public:
  int open(int i2c_bus) {
    sprintf(filename_, "/dev/i2c-%d", i2c_bus);
    if ((file_i2c_ = open(filename_, I2C_RDWR)) < 0) {
      /* Failed to open the i2c bus */
      return -1;
    }
  }

  int read_bytes(int addr, unsigned char buffer, int length) {
    /* Failed to acquire bus access and/or read from slave */
    if (set_address_(addr) && fread(file_i2c_, buffer, length) != length) {
      return -1;
    }
  }

  int write_bytes(int addr, unsigned char buffer, int length) {
    /* Failed to acquire bus access and/or write from slave */
    if (set_address_(addr) && fwrite(file_i2c_, buffer, length) != length) {
      return -1;
    }
  }

private:
  int file_i2c_;
  char *filename_;

  int set_address_(int addr) {
    if (ioctl(file_i2c_, I2C_SLAVE, addr) < 0) {
      return -1;
    }
  }
};
