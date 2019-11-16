#include <drive/drive.hpp>


Drive::Drive() : Node("drive_node") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_variables();

  /* Open UART connection */
  serial::Serial serial_interface(this->serial_port_, this->serial_baudrate_);

  /* Init ROS Publishers and Subscribers */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind(&Drive::command_velocity_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Drive node initialised");
}


void Drive::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("serial_port");
  this->declare_parameter("joint_states_frame");
  this->declare_parameter("odom_frame");
  this->declare_parameter("base_frame");
  this->declare_parameter("wheels.separation");
  this->declare_parameter("wheels.radius");

  // Get parameters from yaml
  this->get_parameter_or<std::string>("serial.port", serial_port_, "/dev/cu.usbserial-AI04Y24G");
  this->get_parameter_or<uint32_t>("serial.baudrate", serial_baudrate_, 115200);

  this->get_parameter_or<std::string>("joint_states_frame", joint_states_.header.frame_id, "base_footprint");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id, "base_footprint");
  this->get_parameter_or<double>("wheels.separation", wheel_separation_, 0.0);
  this->get_parameter_or<double>("wheels.radius", wheel_radius_, 0.0);
}


void Drive::init_variables() {

}


void Drive::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {

}
