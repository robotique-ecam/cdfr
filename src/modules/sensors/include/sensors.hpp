#ifndef SENSORS_HEADER_HPP
#define SENSORS_HEADER_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>

#ifndef SIMULATION
#include "i2c.hpp"
#endif /* SIMULAtION */
#include "sensor_msgs/msg/range.hpp"



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

  // ROS topic publishers
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensors_pub_;

  void init_variables();
  void init_parameters();
};

#endif /* SENSORS_HEADER_HPP */
