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
  sensors_pub_ = this->create_publisher<sensor_msgs::msg::Range>("sensors", qos);

  timer_ = this->create_wall_timer(200ms, std::bind(&Sensors::receive_distance, this));


  RCLCPP_INFO(this->get_logger(), "Sensor node initialised");
}


void Sensors::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("i2c_bus");
  this->declare_parameter("hcsr04_names");

  // Get parameters from yaml
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 3);
  std::vector<std::string> hcsr04_names = this->get_parameter("hcsr04_names").as_string_array();

  for(auto it = hcsr04_names.begin(); it != hcsr04_names.end(); it++) {
     this->declare_parameter("hcsr04." + *it + ".frame_id");
     this->declare_parameter("hcsr04." + *it + ".addr");
     sensor_frames.push_back(this->get_parameter("hcsr04." + *it + ".frame_id").as_string());
     sensor_addresses.push_back(this->get_parameter("hcsr04." + *it + ".addr").as_int());
  }
}


void Sensors::init_variables() {
  hcsr_range_msg.radiation_type = 0;
  hcsr_range_msg.min_range = 0.01;
  hcsr_range_msg.max_range = 2.55;
  hcsr_range_msg.field_of_view = 1;
}


void Sensors::receive_distance() {
  #ifndef SIMULATION
  for (int i=0; i < sensor_addresses.size(); i++) {
    hcsr_range_msg.header.stamp = this->get_clock()->now();
    this->i2c->set_address(sensor_addresses[i]);
    uint8_t distance = this->i2c->read_byte();
    hcsr_range_msg.range = distance / 100;
    sensors_pub_->publish(hcsr_range_msg);
  }
  #endif /* SIMULATION */
}


Sensors::~Sensors() {
  RCLCPP_INFO(this->get_logger(), "Sensors Node Terminated");
}
