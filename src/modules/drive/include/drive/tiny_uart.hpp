#ifndef TINY_UART_HPP
#define TINY_UART_HPP

#include <algorithm>
#include <functional>
#include <inttypes.h>
#include <iostream>
#include <vector>

#define MASTER_ID 0xE
#define BROADCAST_ID 0xF
#define STOP_CONDITION 0xFE
#define START_CONDIION 0xFF

class TinyUART {
public:
  // TinyUART();

  void update(std::vector<uint8_t> *data) {
    serial_data_.insert(serial_data_.end(), data->begin(), data->end());
    do {
      start_condition_index_ = find_index_of(START_CONDIION);
      stop_condition_index_ = find_index_of(STOP_CONDITION);
      switch (stop_condition_index_ - start_condition_index_) {
      case 0: /* No data */
        break;

      case 1: /* Data is ping */
        if (ping_returned_callback != nullptr) {
          (*ping_returned_callback)();
        }
        break;

      case 3: /* Data is steps data */
        uint8_t id = serial_data_[start_condition_index_ + 1] >> 4;
        bool negative_steps = (serial_data_[start_condition_index_ + 1] & 0x4);
        int32_t steps = (serial_data_[start_condition_index_ + 1] & 0x2) << 14 |
                        serial_data_[start_condition_index_ + 2] << 7 |
                        serial_data_[start_condition_index_ + 3];
        if (negative_steps) {
          steps = -steps;
        }
        if (steps_returned_callback != nullptr) {
          (*steps_returned_callback)(steps, id);
        }
        break;
      }
    } while (start_condition_index_ != -1 && stop_condition_index_ != -1);
  }

  void set_steps_callback(std::function<void(int32_t, uint8_t)> callback) {
    steps_returned_callback = &callback;
  }

  void set_ping_callback(std::function<void()> callback) {
    ping_returned_callback = &callback;
  }

private:
  int start_condition_index_ = -1;
  int stop_condition_index_ = -1;
  std::vector<uint8_t> serial_data_;

  std::function<void()> *ping_returned_callback;
  std::function<void(int32_t, uint8_t)> *steps_returned_callback;

  int find_index_of(uint8_t value) {
    auto it = std::find(serial_data_.begin(), serial_data_.end(), value);
    if (it == serial_data_.end()) {
      return -1;
    }
    return std::distance(serial_data_.begin(), it);
  }
};

#endif /* TINY_UART_HPP */
