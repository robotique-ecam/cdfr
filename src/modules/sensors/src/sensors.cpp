#include "sensors.hpp"

#define I2C_ADDR_SENSOR_FRONT_LEFT 0x20
#define I2C_ADDR_SENSOR_FRONT_RIGHT 0x21
#define I2C_ADDR_SENSOR_BACK_LEFT 0x22
#define I2C_ADDR_SENSOR_BACK_RIGHT 0x23

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
  sensors_pub_ = this->create_publisher<sensor_msgs::msg::Range>("sensors",&distance);


  #ifdef USE_TIMER
  timer_ = this->create_wall_timer(50ms, std::bind(&Sensors::receive_distance, this));
  #endif


  RCLCPP_INFO(this->get_logger(), "Sensor node initialised");
}


void Sensors::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("duration");
  this->declare_parameter("distance");

  // Get parameters from yaml
  #ifndef SIMULATION
  this->declare_parameter("i2c_bus");
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 3);
  #endif /* SIMULATION */
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


  this->declare_parameter("hcsr04_names")
std::vector<std::string> n = this->get_parameter("hcsr04_names").as_string_array();

for(std::string &name : n) {
   this->declare_parameter("hcsr04." + name + ".frame_id");
   this->declare_parameter("hcsr04." + name + ".addr");
   int64_t frame_id = this->get_parameter("hcsr04." + name + ".frame_id").();
   int64_t addr = this->get_parameter("hcsr04." + name + ".addr").as_int();
}



  #else
  #endif /* SIMULATION */
}



Sensors::~Sensors() {
  RCLCPP_INFO(this->get_logger(), "Sensors Node Terminated");
}
