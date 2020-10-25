#ifndef SIMULATION

#include <i2c.hpp>

I2C::I2C(int i2c_bus) {
  filename_ = "/dev/i2c-" << std::to_string(i2c_bus);
  i2c_fd_ = open(filename_.c_str(), I2C_RDWR);
}

void I2C::set_address(int addr) { ioctl(i2c_fd_, I2C_SLAVE, addr); }

uint8_t I2C::read_byte() { return i2c_smbus_read_byte(i2c_fd_); }

uint8_t I2C::read_byte_data(uint8_t cmd) { return i2c_smbus_read_byte_data(i2c_fd_, cmd); }

uint16_t I2C::read_word_data(uint8_t cmd) { return i2c_smbus_read_word_data(i2c_fd_, cmd); }

void I2C::write_byte(uint8_t cmd) { i2c_smbus_write_byte(i2c_fd_, cmd); }

void I2C::write_byte_data(uint8_t cmd, uint8_t val) { i2c_smbus_write_byte_data(i2c_fd_, cmd, val); }

void I2C::bus_read(uint8_t cmd, uint8_t *buff) { return i2c_smbus_read_block_data(i2c_fd_, cmd, buff); }

void I2C::bus_write(uint8_t cmd, uint8_t *buff, uint8_t length) { return i2c_smbus_write_block_data(i2c_fd_, cmd, length, buff); }

#endif /* SIMULATION */
