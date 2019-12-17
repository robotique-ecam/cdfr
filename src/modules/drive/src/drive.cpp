#include <drive.hpp>


#define I2C_ADDR_MOTOR_LEFT 0x10
#define I2C_ADDR_MOTOR_RIGHT 0x11


Drive::Drive() : Node("drive_node") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_variables();

  #ifndef SIMULATION
  /* Open I2C connection */
  i2c = std::make_shared<I2C>(i2c_bus);
  #endif /* SIMULATION */
  speedramp_left_ = std::make_shared<Speedramp>();
  speedramp_right_ = std::make_shared<Speedramp>();

  /* Init ROS Publishers and Subscribers */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind(&Drive::command_velocity_callback, this, std::placeholders::_1));

  #ifdef USE_TIMER
  timer_ = this->create_wall_timer(50ms, std::bind(&Drive::update_velocity, this));
  #endif

  RCLCPP_INFO(this->get_logger(), "Drive node initialised");
}


void Drive::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("accel");
  this->declare_parameter("joint_states_frame");
  this->declare_parameter("odom_frame");
  this->declare_parameter("base_frame");
  this->declare_parameter("wheels.separation");
  this->declare_parameter("wheels.radius");
  this->declare_parameter("microcontroler.max_steps_frequency");
  this->declare_parameter("microcontroler.speedramp_resolution");

  // Get parameters from yaml
  #ifndef SIMULATION
  this->declare_parameter("i2c_bus");
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 1);
  #endif /* SIMULATION */
  this->get_parameter_or<std::string>("joint_states_frame", joint_states_.header.frame_id, "base_footprint");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id, "base_footprint");
  this->get_parameter_or<double>("wheels.separation", wheel_separation_, 0.25);
  this->get_parameter_or<double>("wheels.radius", wheel_radius_, 0.080);
  this->get_parameter_or<int>("microcontroler.max_steps_frequency", max_freq_, 10000);
  this->get_parameter_or<int>("microcontroler.speedramp_resolution", speed_resolution_, 128);
  this->get_parameter_or<double>("speedramp.accel", accel_, 9.81);
}


void Drive::init_variables() {
  /* Compute initial values */
  steps_per_turn_ = 200 * 16;
  rads_per_step = 1 / (M_PI * wheel_radius_);
  meters_per_turn_ = 2 * M_PI * wheel_radius_;
  meters_per_step_ = meters_per_turn_ / steps_per_turn_;

  speed_multiplier_ = max_freq_ / speed_resolution_;
  max_speed_ = max_freq_ * meters_per_step_;
  min_speed_ = speed_multiplier_ * meters_per_step_;

  joint_states_.name.push_back("wheel_left_joint");
  joint_states_.name.push_back("wheel_right_joint");
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  #ifdef SIMULATION
  speedramp_left_->set_acceleration(accel_);
  speedramp_right_->set_acceleration(accel_);
  speedramp_left_->set_delay(0.05);
  speedramp_right_->set_delay(0.05);
  speedramp_left_->set_speed_limits(-max_speed_, max_speed_);
  speedramp_right_->set_speed_limits(-max_speed_, max_speed_);

  old_cmd_vel_.right = 0;
  old_cmd_vel_.left = 0;
  #endif /* SIMULATION */

  time_since_last_sync_ = this->get_clock()->now();
}


int8_t Drive::compute_velocity_cmd(double velocity) {
  /* Compute absolute velocity command to be sent to microcontroler */
  double abs_velocity = abs(velocity);
  if (abs_velocity >= max_speed_) {
    return (int8_t) ((velocity<0)?(-1):(1)) * (speed_resolution_ - 1);
  } else if (abs_velocity < min_speed_) {
    return 0;
  } else {
    return (int8_t) round(velocity * (speed_resolution_ - 1) / max_speed_) - ((velocity<0)?(1):(0));
  }
}


void Drive::update_velocity() {
  #ifdef USE_SPEEDRAMP
  double differential_speed_left = speedramp_left_->compute(cmd_vel_.left);
  double differential_speed_right = speedramp_right_->compute(cmd_vel_.right);
  #else
  double differential_speed_left = cmd_vel_.left;
  double differential_speed_right = cmd_vel_.right;
  #endif

  differential_speed_cmd_.left = compute_velocity_cmd(differential_speed_left);
  differential_speed_cmd_.right = compute_velocity_cmd(differential_speed_right);

  #ifndef SIMULATION
  /* Send speed commands */
  this->i2c->set_address(I2C_ADDR_MOTOR_LEFT);
  attiny_steps_returned_.left = (int16_t) this->i2c->read_word(differential_speed_cmd_.left);

  this->i2c->set_address(I2C_ADDR_MOTOR_RIGHT);
  attiny_steps_returned_.right = (int16_t) this->i2c->read_word(differential_speed_cmd_.right);
  #else
  cmd_vel_.right = differential_speed_right;
  cmd_vel_.left = differential_speed_left;
  #endif /* SIMULATION */

  previous_time_since_last_sync_ = time_since_last_sync_;
  time_since_last_sync_ = this->get_clock()->now();
  compute_pose_velocity(attiny_steps_returned_);
  update_odometry();
  update_tf();
  update_joint_states();
}


void Drive::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
  cmd_vel_.left = cmd_vel_msg->linear.x + (cmd_vel_msg->angular.z * wheel_separation_) / 2;
  cmd_vel_.right = cmd_vel_msg->linear.x - (cmd_vel_msg->angular.z * wheel_separation_) / 2;

  update_velocity();
}


void Drive::compute_pose_velocity(TinyData steps_returned) {
  dt = (time_since_last_sync_ - previous_time_since_last_sync_).nanoseconds() * 1e-9;

  #ifndef SIMULATION
  differential_move_.left = meters_per_step_ * steps_returned.left;
  differential_move_.right = meters_per_step_ * steps_returned.right;

  differential_speed_.left = differential_move_.left / dt;
  differential_speed_.right = differential_move_.right / dt;
  #else
  differential_speed_.left = old_cmd_vel_.left;
  differential_speed_.right = old_cmd_vel_.right;

  differential_move_.left = differential_speed_.left * dt;
  differential_move_.right = differential_speed_.right * dt;

  old_cmd_vel_ = cmd_vel_;
  #endif /* SIMULATION */

  instantaneous_move_.linear = (differential_move_.left + differential_move_.right) / 2;
  instantaneous_move_.angular = (differential_move_.left - differential_move_.right) / (wheel_separation_);

  instantaneous_speed_.linear = (differential_speed_.left + differential_speed_.right) / 2;
  instantaneous_speed_.angular = (differential_speed_.left - differential_speed_.right) / (wheel_separation_);

  odom_pose_.x += instantaneous_move_.linear * cos(odom_pose_.thetha + (instantaneous_move_.angular / 2));
  odom_pose_.y += instantaneous_move_.linear * sin(odom_pose_.thetha + (instantaneous_move_.angular / 2));
  odom_pose_.thetha += instantaneous_move_.angular;
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
  joint_states_.position[LEFT] += attiny_steps_returned_.left * rads_per_step;
  joint_states_.position[RIGHT] += attiny_steps_returned_.right * rads_per_step;
  joint_states_.velocity[LEFT] =  attiny_steps_returned_.left * rads_per_step / dt;
  joint_states_.velocity[RIGHT] = attiny_steps_returned_.right * rads_per_step / dt;
  joint_states_pub_->publish(joint_states_);
}


Drive::~Drive() {
  RCLCPP_INFO(this->get_logger(), "Drive Node Terminated");
}
