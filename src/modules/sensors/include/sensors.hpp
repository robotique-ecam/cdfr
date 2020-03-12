#ifndef SENSORS_HEADER_HPP // PQ pas Node
#define SENSORS_HEADER_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <vector>
#ifndef SIMULATION
#include "i2c.hpp"
#include "linux_vl53l1x.hpp"
#endif

using namespace rclcpp;
using namespace std::chrono;

class Sensors : public rclcpp::Node {
public:
  Sensors();
  ~Sensors();

private:
  int i2c_bus;

#ifndef SIMULATION
  // For communicating with sensors over I2C
  std::shared_ptr<I2C> i2c;
#endif /* SIMULAtION */

  std::vector<uint8_t> hcsr04_addresses;
  std::vector<std::string> hcsr04_frames;
  std::vector<uint8_t> vl53l1x_addresses;
  std::vector<std::string> vl53l1x_frames;
  std::vector<LINUX_VL53L1X> vl53l1x_sensors;

  // ROS topic publishers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensors_pub_;
  sensor_msgs::msg::Range hcsr_range_msg;
  sensor_msgs::msg::Range vl53l1x_range_msg;

  void init_variables();
  void init_parameters();
  void init_sensors();
  void receive_distance();
};

#endif /* SENSORS_HEADER_HPP */
