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

  /* Init speed before starting odom */
  this->i2c_mutex.lock();
  this->i2c->set_address(I2C_ADDR_MOTOR_LEFT);
  this->i2c->read_word(0);
  this->i2c->set_address(I2C_ADDR_MOTOR_RIGHT);
  this->i2c->read_word(0);
  this->i2c_mutex.unlock();
#else
  /* Init webots supervisor */
  wb_robot = std::make_shared<webots::Robot>();
  /* Initialize motors */
  wb_left_motor = wb_robot->getMotor("wheel_left_joint");
  wb_right_motor = wb_robot->getMotor("wheel_right_joint");
  wb_left_motor->setPosition(std::numeric_limits<double>::infinity());
  wb_right_motor->setPosition(std::numeric_limits<double>::infinity());
  wb_left_motor->setVelocity(0);
  wb_right_motor->setVelocity(0);
  /* initialize odometry */
  timestep = 1;
  wp_left_encoder = wb_left_motor->getPositionSensor();
  wp_right_encoder = wb_right_motor->getPositionSensor();
  wp_left_encoder->enable(timestep);
  wp_right_encoder->enable(timestep);

  time_stepper_ = this->create_wall_timer(std::chrono::milliseconds((int)timestep), std::bind(&Drive::sim_step, this));
#endif /* SIMULATION */

  /* Init ROS Publishers and Subscribers */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind(&Drive::command_velocity_callback, this, std::placeholders::_1));

  _enable_drivers = this->create_service<std_srvs::srv::SetBool>(
      "enable_drivers", std::bind(&Drive::handle_drivers_enable, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  diagnostics_timer_ = this->create_wall_timer(1s, std::bind(&Drive::update_diagnostic, this));

#ifdef USE_TIMER
  timer_ = this->create_wall_timer(1s, std::bind(&Drive::update_velocity, this));
#endif

  RCLCPP_INFO(this->get_logger(), "Drive node initialised");
}

void Drive::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("accel");
  this->declare_parameter("joint_states_frame");
  this->declare_parameter("odom_frame");
  this->declare_parameter("base_frame");
  this->declare_parameter("steps_per_turn");
  this->declare_parameter("microsteps");
  this->declare_parameter("wheels.separation");
  this->declare_parameter("wheels.radius");
  this->declare_parameter("microcontroler.max_steps_frequency");
  this->declare_parameter("microcontroler.speedramp_resolution");

// Get parameters from yaml
#ifndef SIMULATION
  this->declare_parameter("i2c_bus");
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 1);
#endif /* SIMULATION */
  this->get_parameter_or<std::string>("joint_states_frame", joint_states_.header.frame_id, "base_link");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id, "base_link");
  this->get_parameter_or<int>("steps_per_turn", _steps_per_turn, 200);
  this->get_parameter_or<int>("microsteps", _microsteps, 16);
  this->get_parameter_or<double>("wheels.separation", _wheel_separation, 0.25);
  this->get_parameter_or<double>("wheels.radius", _wheel_radius, 0.080);
  this->get_parameter_or<int>("microcontroler.max_steps_frequency", _max_freq, 10000);
  this->get_parameter_or<int>("microcontroler.speedramp_resolution", _speed_resolution, 128);
  this->get_parameter_or<double>("speedramp.accel", _accel, 9.81);
}

void Drive::init_variables() {
  /* Compute initial values */
  _total_steps_per_turn = _microsteps * _steps_per_turn;
  _rads_per_step = 2 * M_PI / _total_steps_per_turn;
  _meters_per_turn = 2 * M_PI * _wheel_radius;
  _meters_per_step = _meters_per_turn / _total_steps_per_turn;

  _speed_multiplier = _max_freq / _speed_resolution;
  _max_speed = _max_freq * _meters_per_step;
  _min_speed = _speed_multiplier * _meters_per_step;

  joint_states_.name.push_back("wheel_left_joint");
  joint_states_.name.push_back("wheel_right_joint");
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  diagnostics_status_.level = diagnostics_status_.OK;
  diagnostics_status_.name = get_fully_qualified_name();
  diagnostics_status_.message = "Running";
  diagnostics_status_.hardware_id = get_fully_qualified_name();

  diagnostics_array_.status.push_back(diagnostics_status_);
  diagnostics_array_.header.stamp = this->get_clock()->now();

#ifndef SIMULATION
  time_since_last_sync_ = this->get_clock()->now();
#else
  time_since_last_sync_ = get_sim_time();
#endif /* SIMULATION */
}

int8_t Drive::compute_velocity_cmd(double velocity) {
  /* Compute absolute velocity command to be sent to microcontroler */
  double abs_velocity = abs(velocity);
  if (abs_velocity >= _max_speed) {
    return (int8_t)((velocity < 0) ? (-1) : (1)) * (_speed_resolution - 1);
  } else if (abs_velocity < _min_speed) {
    return 0;
  } else {
    return (int8_t)round(velocity * (_speed_resolution - 1) / _max_speed) - ((velocity < 0) ? (1) : (0));
  }
}

void Drive::update_velocity() {
  differential_speed_cmd_.left = compute_velocity_cmd(cmd_vel_.left);
  differential_speed_cmd_.right = compute_velocity_cmd(cmd_vel_.right);

  previous_time_since_last_sync_ = time_since_last_sync_;

#ifndef SIMULATION
  this->i2c_mutex.lock();

  time_since_last_sync_ = this->get_clock()->now();
  /* Send speed commands */
  this->i2c->set_address(I2C_ADDR_MOTOR_LEFT);
  attiny_steps_returned_.left = (int32_t)(this->sign_steps_left ? -1 : 1) * this->i2c->read_word(differential_speed_cmd_.left);
  this->sign_steps_left = signbit(differential_speed_cmd_.left);

  this->i2c->set_address(I2C_ADDR_MOTOR_RIGHT);
  attiny_steps_returned_.right = (int32_t)(this->sign_steps_right ? -1 : 1) * this->i2c->read_word(differential_speed_cmd_.right);
  this->sign_steps_right = signbit(differential_speed_cmd_.right);

  this->i2c_mutex.unlock();

  std::cout << "speed_cmd_.left:" << differential_speed_cmd_.left << " speed_cmd_.right" << differential_speed_cmd_.right << std::endl;
  std::cout << "L" << attiny_steps_returned_.left << " R" << attiny_steps_returned_.right << std::endl;
#else
  time_since_last_sync_ = get_sim_time();

  wb_left_motor->setVelocity(cmd_vel_.left / _wheel_radius);
  wb_right_motor->setVelocity(cmd_vel_.right / _wheel_radius);
  double lsteps = wp_left_encoder->getValue();
  double rsteps = wp_right_encoder->getValue();
  if (std::isnan(lsteps) || std::isnan(rsteps)) {
    lsteps = rsteps = 0;
    RCLCPP_WARN(this->get_logger(), "Robot wheel encoders return NaN");
  }
  attiny_steps_returned_.left = (lsteps - old_steps_returned.left) / _rads_per_step;
  attiny_steps_returned_.right = (rsteps - old_steps_returned.right) / _rads_per_step;
  old_steps_returned.left = lsteps;
  old_steps_returned.right = rsteps;
#endif /* SIMULATION */

  compute_pose_velocity(attiny_steps_returned_);
  update_odometry();
  update_tf();
  update_joint_states();
}

void Drive::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
  cmd_vel_.left = cmd_vel_msg->linear.x - (cmd_vel_msg->angular.z * _wheel_separation) / 2;
  cmd_vel_.right = cmd_vel_msg->linear.x + (cmd_vel_msg->angular.z * _wheel_separation) / 2;

  update_velocity();
}

void Drive::compute_pose_velocity(TinyData steps_returned) {
  dt = (time_since_last_sync_ - previous_time_since_last_sync_).nanoseconds() * 1e-9;

  differential_move_.left = _meters_per_step * steps_returned.left;
  differential_move_.right = _meters_per_step * steps_returned.right;

  differential_speed_.left = differential_move_.left / dt;
  differential_speed_.right = differential_move_.right / dt;

  std::cout << "cmd_vel_.left:" << (int8_t) cmd_vel_.left << " cmd_vel_.right:" << (int8_t) cmd_vel_.right << std::endl;
  std::cout << "speed_.left:" << differential_speed_.left << " speed_.right:" << differential_speed_.right << std::endl;

  instantaneous_move_.linear = (differential_move_.right + differential_move_.left) / 2;
  instantaneous_move_.angular = (differential_move_.right - differential_move_.left) / (_wheel_separation);

  instantaneous_speed_.linear = (differential_speed_.right + differential_speed_.left) / 2;
  instantaneous_speed_.angular = (differential_speed_.right - differential_speed_.left) / (_wheel_separation);

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
  joint_states_.position[LEFT] += attiny_steps_returned_.left * _rads_per_step;
  joint_states_.position[RIGHT] += attiny_steps_returned_.right * _rads_per_step;
  joint_states_.velocity[LEFT] = attiny_steps_returned_.left * _rads_per_step / dt;
  joint_states_.velocity[RIGHT] = attiny_steps_returned_.right * _rads_per_step / dt;
  joint_states_pub_->publish(joint_states_);
}

void Drive::update_diagnostic() { diagnostics_pub_->publish(diagnostics_array_); }

void Drive::handle_drivers_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                  const std_srvs::srv::SetBool::Response::SharedPtr response) {
#ifndef SIMULATION
  this->i2c_mutex.lock();

  this->i2c->set_address(I2C_ADDR_MOTOR_LEFT);
  this->i2c->write_byte_data(STEPPER_CMD, (uint8_t)1 << 4 | request->data);

  this->i2c->set_address(I2C_ADDR_MOTOR_RIGHT);
  this->i2c->write_byte_data(STEPPER_CMD, (uint8_t)1 << 4 | request->data);

  this->i2c_mutex.unlock();
#endif

  if (request->data) {
    response->message = "Stepper drivers are enabled";
    RCLCPP_WARN(this->get_logger(), "Stepper drivers are enabled");
  } else {
    response->message = "Stepper drivers are disabled";
    RCLCPP_WARN(this->get_logger(), "Stepper drivers are disabled");
  }
  response->success = true;
}

#ifdef SIMULATION
rclcpp::Time Drive::get_sim_time() {
  double seconds = 0;
  double nanosec = modf(wb_robot->getTime(), &seconds) * 1e9;
  return rclcpp::Time((uint32_t)seconds, (uint32_t)nanosec);
}

void Drive::sim_step() { this->wb_robot->step(timestep); }
#endif /* SIMULATION */

Drive::~Drive() { RCLCPP_INFO(this->get_logger(), "Drive Node Terminated"); }
