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

  /* Init VL53L1X Sensors */
  init_sensors();

  /* Init ROS Publishers and Subscribers */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  sensors_pub_ = this->create_publisher<sensor_msgs::msg::Range>("sensors", qos);

  timer_ = this->create_wall_timer(200ms, std::bind(&Sensors::receive_distance, this));

  RCLCPP_INFO(this->get_logger(), "Sensor node initialised !");
}

void Sensors::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("i2c_bus");
  this->declare_parameter("vl53l1x_names");

  // Get parameters from yaml
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 4);
  std::vector<std::string> vl53l1x_names = this->get_parameter("vl53l1x_names").as_string_array();

  for (auto name : vl53l1x_names) {
    this->declare_parameter("vl53l1x." + name + ".frame_id");
    this->declare_parameter("vl53l1x." + name + ".addr");
    vl53l1x_frames.push_back(this->get_parameter("vl53l1x." + name + ".frame_id").as_string());
    vl53l1x_addresses.push_back(this->get_parameter("vl53l1x." + name + ".addr").as_int());
  }
}

void Sensors::init_variables() {
  vl53l1x_range_msg.radiation_type = 1;
  vl53l1x_range_msg.min_range = 0.005;
  vl53l1x_range_msg.max_range = 3.5;
  vl53l1x_range_msg.field_of_view = 0.26;
}

void Sensors::init_sensors() {
  for (auto addr : vl53l1x_addresses) {
    vl53l1x_sensors.push_back(LINUX_VL53L1X(i2c.get(), addr));
  }
}

void Sensors::receive_distance() {
#ifndef SIMULATION
  for (auto sensor : vl53l1x_sensors) {
    vl53l1x_range_msg.header.stamp = this->get_clock()->now();
    vl53l1x_range_msg.range = (float)sensor.getDistance() / 1000;
    sensors_pub_->publish(vl53l1x_range_msg);
  }
#endif /* SIMULATION */
}

Sensors::~Sensors() { RCLCPP_INFO(this->get_logger(), "Sensors Node Terminated"); }
