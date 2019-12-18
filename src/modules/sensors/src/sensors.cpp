#include "sensors.hpp"


Sensors::Sensors() : Node("sensors_node") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_variables();

  /* Open I2C connection */
  #ifndef SIMULATION
  i2c = std::make_shared<I2C>(i2c_bus);
  #endif /* SIMULATION */

  /* Init ROS Publishers and Subscribers */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  sensors_pub_ = this->create_publisher<sensor_msgs::msg::Range>("range", qos);

  RCLCPP_INFO(this->get_logger(), "Sensor node initialised");
}


void Sensors::init_parameters() {
  // Declare parameters that may be set on this node
  #ifndef SIMULATION
  this->declare_parameter("i2c_bus");
  #endif /* SIMULATION */

  // Get parameters from yaml
  #ifndef SIMULATION
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 0);
  #endif /* SIMULATION */
}


void Sensors::init_variables() {
  /* Compute initial values */

}

Sensors::~Sensors() {
  RCLCPP_INFO(this->get_logger(), "Sensors Node Terminated");
}
