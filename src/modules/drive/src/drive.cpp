#include <drive.hpp>

#define I2C_ADDR_SLIDER 0x12

Drive::Drive() : Node("drive_node") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_variables();

#ifndef SIMULATION
#ifdef USE_I2C
  /* Open I2C connection */
  i2c = std::make_shared<I2C>(i2c_bus);
#endif//I2C

  odrive = new ODrive(_odrive_usb_port, this);

  /* Init speed before starting odom */
  odrive->setVelocity(0, 0.0);
  odrive->setVelocity(1, 0.0);

  double left_motor_turns_returned, right_motor_turns_returned;
  get_motors_turns_from_odrive(left_motor_turns_returned, right_motor_turns_returned);

  old_motor_turns_returned.left = left_motor_turns_returned;
  old_motor_turns_returned.right = right_motor_turns_returned;

#else
  /* Init webots supervisor */
  std::string namespace_str(Node::get_namespace()), webots_robot_name("WEBOTS_ROBOT_NAME=");
  namespace_str.erase(namespace_str.begin());
  putenv(strdup((webots_robot_name + namespace_str).c_str()));
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
  _set_slider_postion = this->create_service<actuators_srvs::srv::Slider>(
      "slider_position", std::bind(&Drive::handle_set_slider_position, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  _adjust_odometry_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "adjust_odometry", qos, std::bind(&Drive::adjust_odometry_callback, this, std::placeholders::_1));

  diagnostics_timer_ = this->create_wall_timer(1s, std::bind(&Drive::update_diagnostic, this));

#ifdef USE_TIMER
  timer_ = this->create_wall_timer(1s, std::bind(&Drive::update_velocity, this));
#endif

  RCLCPP_INFO(this->get_logger(), "Drive node initialised");
}

void Drive::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("joint_states_frame");
  this->declare_parameter("odom_frame");
  this->declare_parameter("base_frame");
  this->declare_parameter("wheels.separation");
  this->declare_parameter("wheels.radius");

// Get parameters from yaml
#ifndef SIMULATION
#ifdef USE_I2C
  this->declare_parameter("i2c_bus");
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 1);
#endif//I2C
  this->declare_parameter("gearbox_ratio");
  this->declare_parameter("odrive_usb_port");
  this->declare_parameter("invert_wheel");
  this->get_parameter_or<double>("gearbox_ratio", _gearbox_ratio, 1.0);
  this->get_parameter_or<std::string>("odrive_usb_port", _odrive_usb_port, "/dev/ttyS1");
  this->get_parameter_or<bool>("invert_wheel", _invert_wheel, false);
#endif /* SIMULATION */
  this->get_parameter_or<std::string>("joint_states_frame", joint_states_.header.frame_id, "base_link");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id, "base_link");
  this->get_parameter_or<double>("wheels.separation", _wheel_separation, 0.25);
  this->get_parameter_or<double>("wheels.radius", _wheel_radius, 0.080);

}

void Drive::init_variables() {
  /* Compute initial values */
  _wheel_circumference = 2 * M_PI * _wheel_radius;

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
  if (_invert_wheel) {
    odrive_motor_order[0] = 1;
    odrive_motor_order[1] = 0;
  }
  time_since_last_sync_ = this->get_clock()->now();
#else
  time_since_last_sync_ = get_sim_time();
#endif /* SIMULATION */

  tf2_ros::Buffer tfBuffer(std::make_shared<rclcpp::Clock>(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::string errorMsg;
  if(tfBuffer.canTransform(odom_.header.frame_id, "map", std::chrono::system_clock::now(), std::chrono::milliseconds(10000)))
  {
    _map_to_odom_tf = tfBuffer.lookupTransform(odom_.header.frame_id, "map", std::chrono::system_clock::now(), std::chrono::milliseconds(10000));
  }

  geometry_msgs::msg::TransformStamped init_vector;
  init_vector.header.stamp = this->get_clock()->now();
  init_vector.header.frame_id = odom_.header.frame_id;
  init_vector.child_frame_id = odom_.child_frame_id;
  for (int i = 0; i<30; i++) _previous_tf.push_back(init_vector);
}

#ifndef SIMULATION
float Drive::compute_velocity_cmd(double velocity) {
  /* Compute velocity command (in turn/s) to be sent to odrive */
  double omega_wheel = velocity / _wheel_radius;
  double n_wheel = omega_wheel / (2 * M_PI);
  return float(_gearbox_ratio * n_wheel);
}

void Drive::get_motors_turns_from_odrive(double &left, double &right){
  std::pair<float, float> left_turns_speed_returned, right_turns_speed_returned;
  left_turns_speed_returned = odrive->getPosition_Velocity(odrive_motor_order[0]);
  right_turns_speed_returned = odrive->getPosition_Velocity(odrive_motor_order[1]);

  left = double(std::get<0>(left_turns_speed_returned));
  right = double(std::get<0>(right_turns_speed_returned));
}
#endif

void Drive::update_velocity() {

  previous_time_since_last_sync_ = time_since_last_sync_;

#ifndef SIMULATION
  float cmd_odrive_left = compute_velocity_cmd(cmd_vel_.left);
  float cmd_odrive_right = compute_velocity_cmd(cmd_vel_.right);

  time_since_last_sync_ = this->get_clock()->now();
  /* Send speed commands */
  odrive->setVelocity(odrive_motor_order[0], cmd_odrive_left);
  odrive->setVelocity(odrive_motor_order[1], cmd_odrive_right);

  double left_motor_turns_returned, right_motor_turns_returned;
  get_motors_turns_from_odrive(left_motor_turns_returned, right_motor_turns_returned);

  if (left_motor_turns_returned == -1 || right_motor_turns_returned == -1){
    left_motor_turns_returned = old_motor_turns_returned.left;
    right_motor_turns_returned = old_motor_turns_returned.right;
  }

  wheels_turns_returned_.left = (left_motor_turns_returned - old_motor_turns_returned.left) / _gearbox_ratio; //turns
  wheels_turns_returned_.right = (right_motor_turns_returned - old_motor_turns_returned.right) / _gearbox_ratio; //turns
  old_motor_turns_returned.left = left_motor_turns_returned;
  old_motor_turns_returned.right = right_motor_turns_returned;

#else
  time_since_last_sync_ = get_sim_time();
  wb_left_motor->setVelocity(cmd_vel_.left / _wheel_radius);
  wb_right_motor->setVelocity(cmd_vel_.right / _wheel_radius);
  double lturns = wp_left_encoder->getValue(); //rad
  double rturns = wp_right_encoder->getValue(); //rad
  if (std::isnan(lturns) || std::isnan(rturns)) {
    lturns = rturns = 0;
    RCLCPP_WARN(this->get_logger(), "Robot wheel encoders return NaN");
  }
  wheels_turns_returned_.left = (lturns - old_turns_returned.left) / (2 * M_PI); //turns
  wheels_turns_returned_.right = (rturns - old_turns_returned.right) / (2 * M_PI); //turns
  old_turns_returned.left = lturns;
  old_turns_returned.right = rturns;
#endif /* SIMULATION */

  compute_pose_velocity(wheels_turns_returned_);
  update_odometry();
  update_tf();
  update_joint_states();
}

void Drive::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
  cmd_vel_.left = cmd_vel_msg->linear.x - (cmd_vel_msg->angular.z * _wheel_separation) / 2;
  cmd_vel_.right = cmd_vel_msg->linear.x + (cmd_vel_msg->angular.z * _wheel_separation) / 2;

  update_velocity();
}

void Drive::compute_pose_velocity(Differential turns_returned) {
  dt = (time_since_last_sync_ - previous_time_since_last_sync_).nanoseconds() * 1e-9;

  differential_move_.left = turns_returned.left * _wheel_circumference;
  differential_move_.right = turns_returned.right * _wheel_circumference;

  differential_speed_.left = differential_move_.left / dt;
  differential_speed_.right = differential_move_.right / dt;

#ifdef DEBUG
  std::cout << "cmd_vel_.left:" << cmd_vel_.left << " cmd_vel_.right:" << cmd_vel_.right << std::endl;
  std::cout << "speed_.left:" << differential_speed_.left << " speed_.right:" << differential_speed_.right << std::endl;
#endif /* DEBUG */

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

  _previous_tf.insert(_previous_tf.begin(), odom_tf);
  if (_previous_tf.size() > 30) _previous_tf.pop_back();
}

void Drive::update_joint_states() {
  joint_states_.header.stamp = time_since_last_sync_;
  joint_states_.position[LEFT] += wheels_turns_returned_.left * (2 * M_PI);
  joint_states_.position[RIGHT] += wheels_turns_returned_.right * (2 * M_PI);
  joint_states_.velocity[LEFT] = wheels_turns_returned_.left * (2 * M_PI) / dt;
  joint_states_.velocity[RIGHT] = wheels_turns_returned_.right * (2 * M_PI) / dt;
  joint_states_pub_->publish(joint_states_);
}

void Drive::update_diagnostic() { diagnostics_pub_->publish(diagnostics_array_); }

void Drive::handle_drivers_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                  const std_srvs::srv::SetBool::Response::SharedPtr response) {
  if (request->data) {
    response->message = "Stepper drivers are enabled";
    RCLCPP_WARN(this->get_logger(), "Stepper drivers are enabled");
  } else {
    response->message = "Stepper drivers are disabled";
    RCLCPP_WARN(this->get_logger(), "Stepper drivers are disabled");
  }
  response->success = true;
}

void Drive::handle_set_slider_position(const std::shared_ptr<rmw_request_id_t> request_header, const actuators_srvs::srv::Slider::Request::SharedPtr request,
                                       const actuators_srvs::srv::Slider::Response::SharedPtr response) {
#ifdef USE_I2C
  this->i2c_mutex.lock();

  this->i2c->set_address(I2C_ADDR_SLIDER);
  this->i2c->write_byte(request->position);

  this->i2c_mutex.unlock();
#endif
}

#ifdef SIMULATION
rclcpp::Time Drive::get_sim_time() {
  double seconds = 0;
  double nanosec = modf(wb_robot->getTime(), &seconds) * 1e9;
  return rclcpp::Time((uint32_t)seconds, (uint32_t)nanosec);
}

void Drive::sim_step() { this->wb_robot->step(timestep); }
#endif /* SIMULATION */

void Drive::adjust_odometry_callback(const geometry_msgs::msg::TransformStamped::SharedPtr tf_stamped_msg){
  rclcpp::Time stamp_msg = tf_stamped_msg->header.stamp;
  int right_stamp_index = 0;
  while (stamp_msg < (rclcpp::Time)_previous_tf.at(right_stamp_index).header.stamp){
    right_stamp_index++;
    if (right_stamp_index == int(_previous_tf.size())) {
      RCLCPP_WARN(this->get_logger(), "Not enough tf stored to readjust odometry");
      return;
    }
  }

  if (right_stamp_index != 0){
    if (stamp_msg - (rclcpp::Time)_previous_tf[right_stamp_index].header.stamp
      > (rclcpp::Time)_previous_tf[right_stamp_index - 1].header.stamp - stamp_msg) right_stamp_index--;
  }

  geometry_msgs::msg::TransformStamped msg_received_relative_to_odom_tf,
                                       nearest_tf = _previous_tf[right_stamp_index];

  geometry_msgs::msg::PoseStamped msg_received_relative_to_map, msg_received_relative_to_odom,
                                  base_link_relative_to_odom_estim, base_link_relative_to_near,
                                  base_link_relative_to_odom_corrected;

 /* Getting msg_received_relative_to_odom */
  extract_pose_from_transform(*tf_stamped_msg, msg_received_relative_to_map);
  get_pose_in_another_frame(msg_received_relative_to_map, msg_received_relative_to_odom, _map_to_odom_tf);

  /* Getting base_link_relative_to_near */
  extract_pose_from_transform(_previous_tf[0], base_link_relative_to_odom_estim);
  get_pose_in_another_frame(base_link_relative_to_odom_estim, base_link_relative_to_near, nearest_tf);

  /* Getting base_link_relative_to_odom_corrected */
  set_transform_from_pose(msg_received_relative_to_odom, msg_received_relative_to_odom_tf);
  get_pose_in_another_frame(base_link_relative_to_near, base_link_relative_to_odom_corrected, msg_received_relative_to_odom_tf);

  tf2::Quaternion q(
    base_link_relative_to_odom_corrected.pose.orientation.x,
    base_link_relative_to_odom_corrected.pose.orientation.y,
    base_link_relative_to_odom_corrected.pose.orientation.z,
    base_link_relative_to_odom_corrected.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  odom_pose_.thetha = yaw;
  odom_pose_.x = base_link_relative_to_odom_corrected.pose.position.x;
  odom_pose_.y = base_link_relative_to_odom_corrected.pose.position.y;
}

void Drive::extract_pose_from_transform(geometry_msgs::msg::TransformStamped &transform_in, geometry_msgs::msg::PoseStamped &pose_out){
  pose_out.header = transform_in.header;
  pose_out.pose.position.x = transform_in.transform.translation.x;
  pose_out.pose.position.y = transform_in.transform.translation.y;
  pose_out.pose.position.z = transform_in.transform.translation.z;
  pose_out.pose.orientation = transform_in.transform.rotation;
}

void Drive::set_transform_from_pose(geometry_msgs::msg::PoseStamped &pose_in, geometry_msgs::msg::TransformStamped &transform_out){
  transform_out.header.frame_id = odom_.header.frame_id;
  transform_out.child_frame_id = odom_.child_frame_id;
  transform_out.transform.translation.x = pose_in.pose.position.x;
  transform_out.transform.translation.y = pose_in.pose.position.y;
  transform_out.transform.translation.z = pose_in.pose.position.z;
  transform_out.transform.rotation = pose_in.pose.orientation;
}

void Drive::get_pose_in_another_frame(geometry_msgs::msg::PoseStamped &pose_in, geometry_msgs::msg::PoseStamped &pose_out, geometry_msgs::msg::TransformStamped &transform){
  pose_out.header = pose_in.header;
  pose_out.header.frame_id = transform.child_frame_id;

  tf2::Vector3 t_in_new_frame,
               t_pose_in = get_tf2_vector(pose_in.pose.position),
               t_transform = get_tf2_vector(transform.transform.translation);
  tf2::Quaternion q_in_new_frame,
                  q_pose_in = get_tf2_quaternion(pose_in.pose.orientation),
                  q_transform = get_tf2_quaternion(transform.transform.rotation);

  if (pose_in.header.frame_id == transform.header.frame_id){
    t_in_new_frame = t_pose_in - t_transform;
  } else {
    t_in_new_frame = t_pose_in + t_transform;
  }

  if (pose_in.header.frame_id == odom_.child_frame_id || pose_in.header.frame_id == transform.header.frame_id){
    q_in_new_frame = q_transform * q_pose_in.inverse();
  } else {
    q_in_new_frame = q_pose_in * q_transform.inverse();
  }


  set_pose_from_tf_t_q(t_in_new_frame, q_in_new_frame, pose_out);
}

tf2::Quaternion Drive::get_tf2_quaternion(geometry_msgs::msg::Quaternion &q){
  return tf2::Quaternion(q.x,q.y,q.z,q.w);
}

tf2::Vector3 Drive::get_tf2_vector(geometry_msgs::msg::Point &p){
  return tf2::Vector3(p.x, p.y, p.z);
}

tf2::Vector3 Drive::get_tf2_vector(geometry_msgs::msg::Vector3 &p){
  return tf2::Vector3(p.x, p.y, p.z);
}

void Drive::set_pose_from_tf_t_q(tf2::Vector3 &t, tf2::Quaternion &q, geometry_msgs::msg::PoseStamped &pose_out){
  pose_out.pose.position.x = dummy_tree_digits_precision(t.x());
  pose_out.pose.position.y = dummy_tree_digits_precision(t.y());
  pose_out.pose.position.z = 0;
  pose_out.pose.orientation.x = q.x();
  pose_out.pose.orientation.y = q.y();
  pose_out.pose.orientation.z = q.z();
  pose_out.pose.orientation.w = q.w();
}

double Drive::dummy_tree_digits_precision(double a){
  int dummy = (int)(a*1000);
  return (double)(dummy)/1000;
}

Drive::~Drive() {
  RCLCPP_INFO(this->get_logger(), "Drive Node Terminated");
}
