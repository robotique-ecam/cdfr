#ifndef SENSORS_HEADER_HPP    // PQ pas Node
#define SENSORS_HEADER_HPP

#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#ifndef SIMULATION
  #include "i2c.hpp"
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

  std::vector <uint8_t> sensor_addresses;
  std::vector <std::string> sensor_frames;

  // ROS topic publishers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensors_pub_;
  sensor_msgs::msg::Range hcsr_range_msg;


  void init_variables();
  void init_parameters();
  void receive_distance();

};

#endif /* SENSORS_HEADER_HPP */
