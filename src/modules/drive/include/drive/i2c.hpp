#ifndef I2C_HEADER_HPP
#define I2C_HEADER_HPP


#include <cstdio>
#include <string>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>


class I2C {
public:
  I2C(int i2c_bus);
  void set_address(int addr);
  uint8_t read_byte();
  uint16_t read_word(uint8_t cmd);
  void write_byte(uint8_t cmd);

private:
  int i2c_fd_;
  std::string filename_;

};

#endif /* I2C_HEADER_HPP */
