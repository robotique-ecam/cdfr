#include <drive/drive.hpp>


Drive::Drive() : Node("drive_node") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_variables();

  /* Open UART connection */
  try {
    serial::Serial serial_interface(this->serial_port_, this->serial_baudrate_);
  } catch (serial::IOException) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open serial port : %s",  this->serial_port_.c_str());
    exit(1);
  }

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
  this->get_parameter_or<std::string>("serial.port", serial_port_, "/dev/ACM0");
  this->get_parameter_or<uint32_t>("serial.baudrate", serial_baudrate_, 115200);

  this->get_parameter_or<std::string>("joint_states_frame", joint_states_.header.frame_id, "base_footprint");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id, "base_footprint");
  this->get_parameter_or<double>("wheels.separation", wheel_separation_, 0.0);
  this->get_parameter_or<double>("wheels.radius", wheel_radius_, 0.0);
}


void Drive::init_variables() {
  /* Compute initial values */
  steps_per_turn_ = 200 * 16;
  mm_per_turn_ = 2 * M_PI * wheel_radius_;
  mm_per_step_ = mm_per_turn_ / steps_per_turn_;
}


void Drive::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {

}


void Drive::steps_returned_callback(TinyData steps_returned) {
  dt = (time_since_last_sync_ - previous_time_since_last_sync_).nanoseconds() * 1e3;

  differential_move_.left = mm_per_step_ * steps_returned.left;
  differential_move_.right = mm_per_step_ * steps_returned.right;

  differential_speed_.left = differential_move_.left / dt;
  differential_speed_.right = differential_move_.right / dt;

  if (steps_returned.left == steps_returned.right) {

  } else if (steps_returned.left == -steps_returned.right) {

  } else {
    trajectory_radius_left = wheel_separation_ * differential_move_.left / (differential_move_.right - differential_move_.left);
    trajectory_radius = 2 * trajectory_radius_left + wheel_separation_;

    instantaneous_move_.angular = differential_move_.left / trajectory_radius_left;
    instantaneous_move_.linear = wheel_radius_ * sqrt(2 - 2 * cos(instantaneous_move_.angular));

    instantaneous_speed_.angular = instantaneous_move_.angular / dt;
    instantaneous_speed_.linear = instantaneous_move_.linear / dt;
  }
}


Drive::~Drive() {
  RCLCPP_INFO(this->get_logger(), "Drive Node Terminated");
}
