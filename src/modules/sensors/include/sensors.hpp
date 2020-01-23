#ifndef SENSORS_HEADER_HPP    // PQ pas Node
#define SENSORS_HEADER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <vector>
#ifndef SIMULATION
#include "i2c.hpp"
#endif



class Sensors : public rclcpp::Node {
public:
  Sensors();
  ~Sensors();

private:



  #ifndef SIMULATION
  // For communicating with sensors over I2C
  int i2c_bus;
  std::shared_ptr<I2C> i2c;
  #endif /* SIMULAtION */

  std::vector <uint8_t> sensor_addresses {0x20, 0x21, 0x22, 0x23};
  std::vector <std::string> sensor_frames {"sensor_front_left", "sensor_front_right", "sensor_back_left", "sensor_back_right"};

  // ROS topic publishers
  rclcpp:Publisher <sensor_msgs::msg::Range>::SharedPtr sensors_pub_;
  sensor_msgs::msg::Range hcsr_range_msg;


  void init_variables();
  void init_parameters();
  void receive_distance();

};

#endif /* SENSORS_HEADER_HPP */
