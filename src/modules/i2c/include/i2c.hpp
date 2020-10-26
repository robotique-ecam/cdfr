#ifndef I2C_HEADER_HPP
#define I2C_HEADER_HPP

#ifndef SIMULATION

#include <cstdio>
#include <string>
extern "C" {
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}

class I2C {
public:
  I2C(int i2c_bus);
  void set_address(int addr);
  uint8_t read_byte();
  uint8_t read_byte_data(uint8_t cmd);
  uint16_t read_word_data(uint8_t cmd);
  void write_byte(uint8_t cmd);
  void write_byte_data(uint8_t cmd, uint8_t val);
  void bus_read(uint8_t cmd, uint8_t buff[]);
  void bus_write(uint8_t cmd, uint8_t buff[], uint8_t length);

private:
  int i2c_fd_;
  std::string filename_;
};

#endif /* SIMULATION */
#endif /* I2C_HEADER_HPP */
