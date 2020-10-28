#include <transformix.hpp>

Transformix::Transformix() : Node("transformix", "transformix") {
  RCLCPP_INFO(rclcpp::get_logger("transformix"), "Starting the services...");

  service_all = this->create_service<transformix_msgs::srv::TransformixParameters>("transform_poseStamped", &generate_buffer_and_transform_my_pose_stamped);

  service_transform = this->create_service<transformix_msgs::srv::TransformixParametersTransformStamped>("get_transform", &get_transform_stamped_from_frames);

  service_pose = this->create_service<transformix_msgs::srv::TransformixParametersTransfromPose>("get_poseStamped_from_transform", &get_poseStamped_from_transformStamped);

  RCLCPP_INFO(rclcpp::get_logger("transformix"), "Transformix service is started");
}

// very slow transformation, 200ms, but keys in hand, need from_frame, to_frame, pose_in, pose_out
void Transformix::generate_buffer_and_transform_my_pose_stamped(const std::shared_ptr<transformix_msgs::srv::TransformixParameters::Request> request,
                                                                std::shared_ptr<transformix_msgs::srv::TransformixParameters::Response> response) {

  std::string from_frame, to_frame;
  geometry_msgs::msg::PoseStamped poseIn, poseOut;
  geometry_msgs::msg::TransformStamped transformStamped;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();

  from_frame = request->from_frame.data;
  to_frame = request->to_frame.data;
  poseIn = request->pose_in;

  tf2_ros::Buffer tfBuffer(clock, tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));
  tf2_ros::TransformListener tfListener(tfBuffer);

  auto start_total = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 2000000; i++) {
  }
  auto end_total = std::chrono::high_resolution_clock::now();
  double time_taken_total = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - start_total).count();
  time_taken_total *= 1e-9;
#ifdef EXTRALOG
  RCLCPP_INFO(rclcpp::get_logger("transformix"), "Time given to buffer initialisation: %f", time_taken_total);
#endif

  if (canTransform(tfBuffer, from_frame, to_frame)) {
    get_transform(tfBuffer, from_frame, to_frame, transformStamped);
    tf2::doTransform<geometry_msgs::msg::PoseStamped>(poseIn, poseOut, transformStamped);
  } else {
    setPose(poseOut, to_frame, 0, 0, 0, 1, 0, 0, 0);
  }

  response->pose_out = poseOut;
}

void Transformix::get_transform_stamped_from_frames(const std::shared_ptr<transformix_msgs::srv::TransformixParametersTransformStamped::Request> request,
                                                    std::shared_ptr<transformix_msgs::srv::TransformixParametersTransformStamped::Response> response) {

  std::string from_frame, to_frame;
  geometry_msgs::msg::TransformStamped transformStamped;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();

  from_frame = request->from_frame.data;
  to_frame = request->to_frame.data;

  tf2_ros::Buffer tfBuffer(clock, tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));
  tf2_ros::TransformListener tfListener(tfBuffer);

  auto start_total = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 2000000; i++) {
  }
  auto end_total = std::chrono::high_resolution_clock::now();
  double time_taken_total = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - start_total).count();
  time_taken_total *= 1e-9;
#ifdef EXTRALOG
  RCLCPP_INFO(rclcpp::get_logger("transformix"), "Time given to buffer initialisation: %f", time_taken_total);
#endif

  if (canTransform(tfBuffer, from_frame, to_frame)) {
    get_transform(tfBuffer, from_frame, to_frame, transformStamped);
  } else {
    setTransform(transformStamped, to_frame, from_frame, 0, 0, 0, 1, 0, 0, 0);
  }

  response->transform_stamped = transformStamped;
}

void Transformix::get_poseStamped_from_transformStamped(const std::shared_ptr<transformix_msgs::srv::TransformixParametersTransfromPose::Request> request,
                                                        std::shared_ptr<transformix_msgs::srv::TransformixParametersTransfromPose::Response> response) {

  geometry_msgs::msg::PoseStamped poseIn, poseOut;
  geometry_msgs::msg::TransformStamped transformStamped;

  poseIn = request->pose_in;
  transformStamped = request->transform_stamped;

  tf2::doTransform<geometry_msgs::msg::PoseStamped>(poseIn, poseOut, transformStamped);

  response->pose_out = poseOut;
}

void Transformix::get_transform(tf2_ros::Buffer &tfBuffer, std::string &from_frame, std::string &to_frame, geometry_msgs::msg::TransformStamped &transformStampedOut) {
#ifdef EXTRALOG
  RCLCPP_INFO(rclcpp::get_logger("transformix"), "We're trying to get the transformation");
#endif

  try {
    transformStampedOut = tfBuffer.lookupTransform(to_frame, from_frame, std::chrono::system_clock::now(), std::chrono::milliseconds(500));
#ifdef EXTRALOG
    RCLCPP_INFO(rclcpp::get_logger("transformix"), "Transformation get");
#endif
  } catch (tf2::TransformException &ex) {
    RCLCPP_INFO(rclcpp::get_logger("transformix"), ex.what());
  }
}

bool Transformix::canTransform(tf2_ros::Buffer &tfBuffer, std::string &from_frame, std::string &to_frame) {
#ifdef EXTRALOG
  RCLCPP_INFO(rclcpp::get_logger("transformix"), "Is The transformation possible ?");
#endif
  try {
    auto start_total = std::chrono::high_resolution_clock::now();

    bool transformation_possible = tfBuffer.canTransform(to_frame, from_frame, std::chrono::system_clock::now(), std::chrono::milliseconds(500), NULL);

    auto end_total = std::chrono::high_resolution_clock::now();
    double time_taken_total = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - start_total).count();
    time_taken_total *= 1e-9;
#ifdef EXTRALOG
    RCLCPP_INFO(rclcpp::get_logger("transformix"), "Time of transformation: %f", time_taken_total);
    if (transformation_possible) {
      RCLCPP_INFO(rclcpp::get_logger("transformix"), "True");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("transformix"), "false");
    }
#endif
    return transformation_possible;
  } catch (tf2::TransformException &ex) {
    return false;
  }
}

void Transformix::setPose(geometry_msgs::msg::PoseStamped &pose, std::string frame, double posx, double posy, double posz, double rotw, double rotx, double roty, double rotz) {
  pose.header.stamp.sec = 1;
  pose.header.stamp.nanosec = 0;
  pose.header.frame_id = frame;
  pose.pose.position.x = posx;
  pose.pose.position.y = posy;
  pose.pose.position.z = posz;
  pose.pose.orientation.w = rotw;
  pose.pose.orientation.x = rotx;
  pose.pose.orientation.y = roty;
  pose.pose.orientation.z = rotz;
}

void Transformix::setTransform(geometry_msgs::msg::TransformStamped &transform, std::string frame_id, std::string child_frame_id, double posx, double posy, double posz,
                               double rotw, double rotx, double roty, double rotz) {
  transform.header.stamp.sec = 1;
  transform.header.stamp.nanosec = 0;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.translation.x = posx;
  transform.transform.translation.y = posy;
  transform.transform.translation.z = posz;
  transform.transform.rotation.w = rotw;
  transform.transform.rotation.x = rotx;
  transform.transform.rotation.y = roty;
  transform.transform.rotation.z = rotz;
}

Transformix::~Transformix() { RCLCPP_INFO(rclcpp::get_logger("transformix"), "Transformix node terminated"); }

/* some unused functions
message_filters::Subscriber<geometry_msgs::msg::TransformStamped> subscriber(this, (std::string)"/tf", rmw_qos_profile_default);
tf2_ros::Buffer tfBuffer(this->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));
tf2_ros::MessageFilter<geometry_msgs::msg::TransformStamped> msgFilter(subscriber, tfBuffer, to_frame, 50, this->get_node_logging_interface(),
    this->get_node_clock_interface(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));

tf2_ros::TransformListener tfListener(tfBuffer);
timerInterface->createTimer(this->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), callback);

transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(to_topic, qos);

while(true){
  if (canTransform(tfBuffer)){ transform(tfBuffer); }
} */
