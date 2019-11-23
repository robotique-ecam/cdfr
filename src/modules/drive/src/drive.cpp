#include <drive/drive.hpp>


Drive::Drive() : Node("drive_node") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_variables();

  /* Open UART connection */
  try {
    serial_interface_ = std::make_shared<serial::Serial>(this->serial_port_, this->serial_baudrate_);
  } catch (serial::IOException) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open serial port : %s",  this->serial_port_.c_str());
    exit(1);
  }
  // Test the timeout at 2ms
  // serial_interface_.setTimeout(serial::Timeout::max(), 0, 2, 0, 2);

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
  this->get_parameter_or<std::string>("serial.port", serial_port_, "/dev/ttyUSB0");
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
  uint8_t differential_speed_left = cmd_vel_msg->linear.x - (cmd_vel_msg->angular.z * wheel_separation_) / 2;
  uint8_t differential_speed_right = cmd_vel_msg->linear.x + (cmd_vel_msg->angular.z * wheel_separation_) / 2;

  /* Set first bit of the ID according to differential_speed_cmd_ sign */
  differential_speed_cmd_.left[2] = differential_speed_left;
  differential_speed_cmd_.right[2] = differential_speed_right;
  differential_speed_cmd_.left[1] ^= (-signbit(differential_speed_left) ^ differential_speed_cmd_.left[1]) & 1;
  differential_speed_cmd_.right[1] ^= (-signbit(differential_speed_cmd_.right[1]) ^ differential_speed_cmd_.right[1]) & 1;

  /* Send speed commands */
  this->serial_interface_->write((const uint8_t*) differential_speed_cmd_.left, sizeof(differential_speed_cmd_.left));
  this->serial_interface_->write((const uint8_t*) differential_speed_cmd_.right, sizeof(differential_speed_cmd_.right));

  previous_time_since_last_sync_ = time_since_last_sync_;
  time_since_last_sync_ = this->now();
}


void Drive::compute_pose_velocity(TinyData steps_returned) {
  dt = (time_since_last_sync_ - previous_time_since_last_sync_).nanoseconds() * 1e3;

  differential_move_.left = mm_per_step_ * steps_returned.left;
  differential_move_.right = mm_per_step_ * steps_returned.right;

  differential_speed_.left = differential_move_.left / dt;
  differential_speed_.right = differential_move_.right / dt;

  if (steps_returned.left == steps_returned.right) {
    instantaneous_speed_.angular = 0;
    instantaneous_speed_.linear = differential_speed_.left;
  } else if (steps_returned.left == -steps_returned.right) {
    instantaneous_speed_.linear = 0;
    instantaneous_speed_.angular = (differential_speed_.right - differential_move_.left) / wheel_separation_;
  } else {
    trajectory_radius_left = wheel_separation_ * differential_move_.left / (differential_move_.right - differential_move_.left);
    trajectory_radius = 2 * trajectory_radius_left + wheel_separation_;

    instantaneous_move_.angular = differential_move_.left / trajectory_radius_left;
    instantaneous_move_.linear = wheel_radius_ * sqrt(2 - 2 * cos(instantaneous_move_.angular));

    instantaneous_speed_.angular = instantaneous_move_.angular / dt;
    instantaneous_speed_.linear = instantaneous_move_.linear / dt;
  }
}


void Drive::update_odometry() {
  odom_.pose.pose.position.x = odom_pose_.x;
  odom_.pose.pose.position.y = odom_pose_.y;
  odom_.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose_.thetha);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x = instantaneous_speed_.linear;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.angular.z = instantaneous_speed_.angular;
  odom_.header.stamp = time_since_last_sync_;
  odom_pub_->publish(odom_);
}


void Drive::update_tf() {
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;

  tf2_msgs::msg::TFMessage odom_tf_msg;
  odom_tf_msg.transforms.push_back(odom_tf);
  tf_pub_->publish(odom_tf_msg);
}

void Drive::update_joint_states() {
  joint_states_.header.stamp = time_since_last_sync_;
  joint_states_.position[LEFT];
  joint_states_.position[RIGHT];
  joint_states_.velocity[LEFT];
  joint_states_.velocity[RIGHT];
  joint_states_pub_->publish(joint_states_);
}


Drive::~Drive() {
  RCLCPP_INFO(this->get_logger(), "Drive Node Terminated");
}
