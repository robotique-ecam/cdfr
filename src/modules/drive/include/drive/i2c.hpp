#ifndef I2C_HEADER_HPP
#define I2C_HEADER_HPP


#include <cstdio>
#include <string>
#include <fcntl.h>
extern "C" {
  #include <sys/stat.h>
  #include <sys/ioctl.h>
  #include <sys/types.h>
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}

extern __s32 i2c_smbus_write_quick(int file, __u8 value);
extern __s32 i2c_smbus_read_byte(int file);
extern __s32 i2c_smbus_write_byte(int file, __u8 value);
extern __s32 i2c_smbus_read_byte_data(int file, __u8 command);
extern __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);
extern __s32 i2c_smbus_read_word_data(int file, __u8 command);
extern __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);
extern __s32 i2c_smbus_process_call(int file, __u8 command, __u16 value);


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
