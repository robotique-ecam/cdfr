#include <assurancetourix.hpp>

Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
    for (auto &new_parameter : event->new_parameters) {
      if (new_parameter.name == "side" && (new_parameter.value.string_value == "blue" || new_parameter.value.string_value == "yellow")) {
        RCLCPP_INFO(this->get_logger(), "new_parameter from node %s, side = %s \n", event->node.c_str(), new_parameter.value.string_value.c_str());
        side = new_parameter.value.string_value;
      }
    }

    for (auto &changed_parameter : event->changed_parameters) {
      if (changed_parameter.name == "side" && (changed_parameter.value.string_value == "blue" || changed_parameter.value.string_value == "yellow")) {
        RCLCPP_INFO(this->get_logger(), "changed_parameter from node %s, side = %s \n", event->node.c_str(), changed_parameter.value.string_value.c_str());
        side = changed_parameter.value.string_value;
      }
    }
  };

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

  transformClient = this->create_client<transformix_msgs::srv::TransformixParametersTransformStamped>("transformix/get_transform");
  getTransformation(assurancetourix_to_map_transformation);

#ifdef CAMERA

  RCLCPP_INFO(this->get_logger(), "Starting CAMERA mode");

  init_camera_settings();

  timer_ = NULL;

  _enable_aruco_detection = this->create_service<std_srvs::srv::SetBool>(
      "/enable_aruco_detection", std::bind(&Assurancetourix::handle_aruco_detection_enable, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

#endif // CAMERA

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_aruco_position", qos);
  transformed_marker_pub_ennemies_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_for_gradient_layer, qos);
  transformed_marker_pub_allies_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(allies_positions_topic, qos);

  if (show_image) {
    cv::namedWindow("anotated", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
  }
  savedeee = false;

#ifdef SIMULATION
  RCLCPP_INFO(this->get_logger(), "Starting SIMULATION mode");
  RCLCPP_INFO(this->get_logger(), "Simulated robots:");
  for (auto robot : robots) {
    RCLCPP_INFO(this->get_logger(), "%s", robot.c_str());
  }

  char mypath[] = "WEBOTS_ROBOT_NAME=game_manager";
  putenv(mypath);

  wb_supervisor = std::make_shared<webots::Supervisor>();
  comeback = false;

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / refresh_frequency), std::bind(&Assurancetourix::simulation_marker_callback, this));
#endif
  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
}

#ifdef CAMERA

void Assurancetourix::init_camera_settings(){
  _cap.open(2, cv::CAP_V4L2);
  _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

  this->declare_parameter("camera_settings.width");
  this->declare_parameter("camera_settings.height");
  this->declare_parameter("camera_settings.brightness");
  this->declare_parameter("camera_settings.contrast");
  this->declare_parameter("camera_settings.saturation");
  this->declare_parameter("camera_settings.hue");
  this->declare_parameter("camera_settings.gamma");
  this->declare_parameter("camera_settings.gain");
  this->declare_parameter("camera_settings.backlight_compensation");
  this->declare_parameter("camera_settings.auto_WB");
  this->declare_parameter("camera_settings.exposure");

  this->get_parameter<int>("camera_settings.width", _camera_settings.width);
  this->get_parameter<int>("camera_settings.height", _camera_settings.height);
  this->get_parameter<int>("camera_settings.brightness", _camera_settings.brightness);
  this->get_parameter<int>("camera_settings.contrast", _camera_settings.contrast);
  this->get_parameter<int>("camera_settings.saturation", _camera_settings.saturation);
  this->get_parameter<int>("camera_settings.hue", _camera_settings.hue);
  this->get_parameter<int>("camera_settings.gamma", _camera_settings.gamma);
  this->get_parameter<int>("camera_settings.gain", _camera_settings.gain);
  this->get_parameter<int>("camera_settings.backlight_compensation", _camera_settings.backlight_compensation);
  this->get_parameter<bool>("camera_settings.auto_WB", _camera_settings.auto_WB);
  this->get_parameter<int>("camera_settings.exposure", _camera_settings.exposure);

  _cap.set(cv::CAP_PROP_FRAME_WIDTH, _camera_settings.width);
  _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _camera_settings.height);
  _cap.set(cv::CAP_PROP_BRIGHTNESS, _camera_settings.brightness);
  _cap.set(cv::CAP_PROP_CONTRAST, _camera_settings.contrast);
  _cap.set(cv::CAP_PROP_SATURATION, _camera_settings.saturation);
  _cap.set(cv::CAP_PROP_HUE, _camera_settings.hue);
  _cap.set(cv::CAP_PROP_GAMMA, _camera_settings.gamma);
  _cap.set(cv::CAP_PROP_GAIN, _camera_settings.gain);
  _cap.set(cv::CAP_PROP_BACKLIGHT, _camera_settings.backlight_compensation);
  _cap.set(cv::CAP_PROP_AUTO_WB, _camera_settings.auto_WB);
  _cap.set(cv::CAP_PROP_EXPOSURE, _camera_settings.exposure);
}

void Assurancetourix::get_image() {
  //_cap.read(_anotated);
  _anotated = imread("/home/phileas/vision/calibration/undistorded_results/board_overview.jpg", IMREAD_GRAYSCALE);
}

// service enabling/disabling aruco detection, disabled by default
// service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
void Assurancetourix::handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                                    const std_srvs::srv::SetBool::Response::SharedPtr response) {

  if (request->data) {
    timer_ = this->create_wall_timer(0.1s, std::bind(&Assurancetourix::detect, this));
    response->message = "Aruco detection is enabled";
    RCLCPP_WARN(this->get_logger(), "Aruco detection is enabled");
  } else {
    timer_ = NULL;
    response->message = "Aruco detection is disabled";
    RCLCPP_WARN(this->get_logger(), "Aruco detection is disabled");
  }
  response->success = true;
}
#endif // CAMERA

#ifdef SIMULATION
// webots element positioning
void Assurancetourix::simulation_marker_callback() {
  visualization_msgs::msg::MarkerArray marker_array_ennemies, marker_array_allies;
  visualization_msgs::msg::Marker webots_marker;
  webots_marker.type = robot_type;
  webots_marker.color.r = 255;
  webots_marker.color.a = 1;
  webots_marker.scale.x = 0.1;
  webots_marker.scale.y = 0.1;
  webots_marker.scale.z = 0.1;
  webots_marker.header.frame_id = "map";
  int id = 0;

  for (auto robot : robots) {
    double x, y;
    double theta = acos(wb_supervisor->getFromDef(robot)->getOrientation()[0]);

    if (theta < 0) theta = -theta;
    if (wb_supervisor->getFromDef(robot)->getOrientation()[1] > 0) theta = -theta;

    x = wb_supervisor->getFromDef(robot)->getPosition()[0];
    y = 2 - wb_supervisor->getFromDef(robot)->getPosition()[2];

    webots_marker.header.stamp = get_sim_time(wb_supervisor);
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    webots_marker.pose.orientation.x = q.x();
    webots_marker.pose.orientation.y = q.y();
    webots_marker.pose.orientation.z = q.z();
    webots_marker.pose.orientation.w = q.w();

    webots_marker.pose.position.x = x;
    webots_marker.pose.position.y = y;

    webots_marker.text = robot;
    webots_marker.id = id;

    marker_array_allies.markers.push_back(webots_marker);
    id++;
  }
  id = 6;
  webots_marker.pose.orientation.x = 0.0;
  webots_marker.pose.orientation.y = 0.0;
  webots_marker.pose.orientation.z = 0.0;
  webots_marker.pose.orientation.w = 1.0;
  webots_marker.header.stamp = this->get_clock()->now();
  if (comeback) {
    comeback_x += 0.1;
    if (comeback_x > 2.5){
      comeback = false;
    }
  } else {
    comeback_x -= 0.1;
    if (comeback_x < 0.5){
      comeback = true;
    }
  }
  webots_marker.pose.position.x = comeback_x;

  webots_marker.pose.position.y = 1;
  webots_marker.text = "test";
  webots_marker.id = id;
  marker_array_ennemies.markers.push_back(webots_marker);
  marker_array_ennemies.markers.push_back(predictEnnemiesPos(webots_marker));

  transformed_marker_pub_ennemies_->publish(marker_array_ennemies);
  lastEnnemiesMarkers = ennemiesMarkersOnThisCycle;
  ennemiesMarkersOnThisCycle.markers.clear();

  transformed_marker_pub_allies_->publish(marker_array_allies);
}
#endif // SIMULATION

visualization_msgs::msg::Marker Assurancetourix::predictEnnemiesPos(visualization_msgs::msg::Marker detectedMarker){
  ennemiesMarkersOnThisCycle.markers.push_back(detectedMarker);
  for(int i = 0; i< int(lastEnnemiesMarkers.markers.size()); i++ ){
    if (lastEnnemiesMarkers.markers[i].id == detectedMarker.id){
      visualization_msgs::msg::Marker predictedMarker = detectedMarker;
      predictedMarker.pose.position.x += detectedMarker.pose.position.x - lastEnnemiesMarkers.markers[i].pose.position.x;
      predictedMarker.pose.position.y += detectedMarker.pose.position.y - lastEnnemiesMarkers.markers[i].pose.position.y;
      predictedMarker.id = detectedMarker.id + 10;
      predictedMarker.header.stamp = this->get_clock()->now();
      predictedMarker.color.r = 0;
      predictedMarker.color.g = 255;
      return predictedMarker;
    }
  }
  return detectedMarker;
}

void Assurancetourix::init_parameters() {

  // declare variables from yml
  this->declare_parameter("image_post_processing.show_image");
  this->declare_parameter("image_post_processing.small_aruco_size");
  this->declare_parameter("image_post_processing.huge_aruco_size");
  this->get_parameter_or<bool>("image_post_processing.show_image", show_image, false);
  this->get_parameter_or<double>("image_post_processing.small_aruco_size", small_aruco_size, 0.07);
  this->get_parameter_or<double>("image_post_processing.huge_aruco_size", huge_aruco_size, 0.115);

#ifdef SIMULATION
  this->declare_parameter("simulation.robots");
  this->declare_parameter("simulation.refresh_frequency");
  robots = this->get_parameter("simulation.robots").as_string_array();
  this->get_parameter_or<int>("simulation.refresh_frequency", refresh_frequency, 5);
#endif // SIMULATION

  this->declare_parameter("rviz_settings.blue_color_ArUco");
  this->declare_parameter("rviz_settings.yellow_color_ArUco");
  this->declare_parameter("rviz_settings.default_color_ArUco");
  this->declare_parameter("rviz_settings.arrow_scale");
  this->declare_parameter("rviz_settings.game_elements_scale");
  this->declare_parameter("rviz_settings.robot_type");
  this->declare_parameter("rviz_settings.game_element_type");
  this->declare_parameter("rviz_settings.lifetime_sec");
  this->declare_parameter("rviz_settings.header_frame_id");
  this->declare_parameter("topic_for_gradient_layer");
  this->declare_parameter("topic_for_allies_position");
  this->declare_parameter("side");

  this->get_parameter_or<std::vector<double>>("rviz_settings.blue_color_ArUco", blue_color_ArUco, {0.0, 0.0, 255.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.yellow_color_ArUco", yellow_color_ArUco, {255.0, 255.0, 0.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.default_color_ArUco", default_color_ArUco, {120.0, 120.0, 120.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.arrow_scale", arrow_scale, {0.05, 0.05, 0.05});
  this->get_parameter_or<std::vector<double>>("rviz_settings.game_elements_scale", game_elements_scale, {0.07, 0.07, 0.01});
  this->get_parameter_or<uint>("rviz_settings.robot_type", robot_type, 2);
  this->get_parameter_or<uint>("rviz_settings.game_element_type", game_element_type, 1);
  this->get_parameter_or<int>("rviz_settings.lifetime_sec", lifetime_sec, 2);
  this->get_parameter_or<std::string>("rviz_settings.header_frame_id", header_frame_id, "assurancetourix");
  this->get_parameter_or<std::string>("topic_for_gradient_layer", topic_for_gradient_layer, "/default_topic_for_gradient_layer");
  this->get_parameter_or<std::string>("topic_for_allies_position", allies_positions_topic, "/default_topic_for_allies_positions");
  this->get_parameter_or<std::string>("side", side, "blue");

  // initialisation of markers
  marker.header.frame_id = header_frame_id;
  marker.color.a = 1.0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime.sec = lifetime_sec;

  base_frame = "map";
  transformed_marker.header.frame_id = base_frame;
  transformed_marker.color.a = 1.0;
  transformed_marker.action = visualization_msgs::msg::Marker::ADD;
  transformed_marker.lifetime.sec = lifetime_sec;
}

void Assurancetourix::detect() {

#ifdef EXTRALOG // if enabled show the time elapsed to do some actions (capture the image, find the coordonates of the arucos in it and the total)
  // will also show a marker at the assurancetourix origin frame position
  marker.pose.orientation.x = 0.70711;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0.70711;
  marker.pose.orientation.w = 0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.type = robot_type;
  marker.color.r = 255;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.id = 13;

  auto start_camera = std::chrono::high_resolution_clock::now();
  std::ios_base::sync_with_stdio(false);
#endif

#ifdef CAMERA
  get_image();
#endif // CAMERA


#ifdef EXTRALOG
  auto end_camera = std::chrono::high_resolution_clock::now();
#endif

  // cv::resize(_frame, _frame, Size(), 0.5, 0.5, cv::INTER_LINEAR);
  //_frame.convertTo(raised_contrast, -1, contrast, 0);

  //raised_contrast.copyTo(_anotated);

#ifdef EXTRALOG
  marker.header.stamp = this->get_clock()->now();
  transformed_marker.header.stamp = this->get_clock()->now();

  marker_pub_->publish(marker);

  auto start_detection = std::chrono::high_resolution_clock::now();
#endif

  _detect_aruco(_anotated);
  estimate_arucos_poses();

#ifdef EXTRALOG
  auto end_detection = std::chrono::high_resolution_clock::now();
  auto end_total = std::chrono::high_resolution_clock::now();

  double time_taken_camera = std::chrono::duration_cast<std::chrono::nanoseconds>(end_camera - start_camera).count();
  double time_taken_detection = std::chrono::duration_cast<std::chrono::nanoseconds>(end_detection - start_detection).count();
  double time_taken_total = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - start_camera).count();

  time_taken_camera *= 1e-9;
  time_taken_detection *= 1e-9;
  time_taken_total *= 1e-9;

  RCLCPP_INFO(this->get_logger(), "time of camera: %f", time_taken_camera);
  RCLCPP_INFO(this->get_logger(), "time of detection: %f", time_taken_detection);
  RCLCPP_INFO(this->get_logger(), "time of total: %f", time_taken_total);
#endif

  if (show_image) {
    cv::resize(tmp, tmp, Size(), 0.4, 0.4, cv::INTER_LINEAR);
    cv::imshow("anotated", tmp);
    // cv::imshow("origin", _frame);
    cv::waitKey(3);
  }

  if (!savedeee) {
    cv::imwrite("test.jpg", _anotated);
    savedeee = true;
  }
}

void Assurancetourix::getTransformation(geometry_msgs::msg::TransformStamped &transformation) {
  auto request = std::make_shared<transformix_msgs::srv::TransformixParametersTransformStamped::Request>();

  std_msgs::msg::String fromFrame, toFrame;
  fromFrame.data = header_frame_id;
  toFrame.data = base_frame;
  request->from_frame = fromFrame;
  request->to_frame = toFrame;

  while (!transformClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto result = transformClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    transformation = result.get()->transform_stamped;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transformation");
  }
}

void Assurancetourix::compute_final_projection(std::vector<std::vector<cv::Point2f>> &coordinates_vector, std::vector<cv::Point2f> &undistort){
  coordinates_vector.push_back({ {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} });
  for(int j=0; j<4; j++){
    coordinates_vector.back()[j] = {
        undistort[j].x * mat_camera_matrix_coeff_fisheye_balanced[0][0] +
        mat_camera_matrix_coeff_fisheye_balanced[0][2],
        undistort[j].y * mat_camera_matrix_coeff_fisheye_balanced[1][1] +
        mat_camera_matrix_coeff_fisheye_balanced[1][2]
      };
  }
}

void Assurancetourix::project_corners_pinhole_to_fisheye(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> detected_ids) {
  _small_marker_corners_projection.clear();
  _huge_marker_corners_projection.clear();
  _small_detected_ids.clear();
  _huge_detected_ids.clear();
  for(int i=0; i<int(detected_ids.size()); i++){
    std::vector<cv::Point2f> undistort;
    cv::fisheye::undistortPoints(marker_corners[i], undistort, _cameraMatrix_fisheye, _distCoeffs_fisheye);
    if(detected_ids[i]<50){
      _small_detected_ids.push_back(detected_ids[i]);
      compute_final_projection(_small_marker_corners_projection, undistort);
    } else {
      _huge_detected_ids.push_back(detected_ids[i]);
      compute_final_projection(_huge_marker_corners_projection, undistort);
    }
  }
}

void Assurancetourix::_detect_aruco(Mat img) {
  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  std::vector<int> detected_ids;
  cv::aruco::detectMarkers(img, _dictionary, marker_corners, detected_ids, _parameters, rejected_candidates);
  if (detected_ids.size() > 0){
    project_corners_pinhole_to_fisheye(marker_corners, detected_ids);
    if (show_image) {
      tmp = imread("/home/phileas/vision/calibration/undistorded_results/200_pictures_fisheye/board_overview_undistord_balance_0_3.jpg");
      // drawDetectedMarkers on the image (activate only for debug)
      cv::aruco::drawDetectedMarkers(tmp, _small_marker_corners_projection, _small_detected_ids);
      cv::aruco::drawDetectedMarkers(tmp, _huge_marker_corners_projection, _huge_detected_ids);
    }
  }
}

void Assurancetourix::set_vision_for_rviz(std::vector<double> color, std::vector<double> scale, uint type) {
  marker.type = type;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.scale.x = scale[0];
  marker.scale.y = scale[1];
  marker.scale.z = scale[2];
  transformed_marker.type = type;
  transformed_marker.color.r = color[0];
  transformed_marker.color.g = color[1];
  transformed_marker.color.b = color[2];
  transformed_marker.scale.x = scale[0];
  transformed_marker.scale.y = scale[1];
  transformed_marker.scale.z = scale[2];
}

void Assurancetourix::_anotate_image(Mat img) {
  if (_detected_ids.size() > 0) {
    visualization_msgs::msg::MarkerArray marker_array_ennemies, marker_array_allies;

    cv::aruco::estimatePoseSingleMarkers(_marker_corners_projection, 0.115, _cameraMatrix_pinhole, _distCoeffs_pinhole, _rvecs, _tvecs);

    for (int i = 0; i < int(_detected_ids.size()); i++) {

      if (show_image) {
        // drawAxis on the image (activate only for debug)
        cv::aruco::drawAxis(img, _cameraMatrix_pinhole, _distCoeffs_pinhole, _rvecs[i], _tvecs[i], 0.1);
      }

      marker.pose.position.x = _tvecs[i].operator[](0);
      marker.pose.position.y = _tvecs[i].operator[](1);
      marker.pose.position.z = _tvecs[i].operator[](2);

      double angle = norm(_rvecs[i]);
      Vec3d axis = _rvecs[i] / angle;

      tf2::Quaternion q;
      q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      if ((_detected_ids[i] <= 5) && (1 <= _detected_ids[i])) {
        set_vision_for_rviz(blue_color_ArUco, arrow_scale, robot_type);
      } else if ((_detected_ids[i] <= 10) && (6 <= _detected_ids[i])) {
        set_vision_for_rviz(yellow_color_ArUco, arrow_scale, robot_type);
      } else {
        set_vision_for_rviz(default_color_ArUco, game_elements_scale, game_element_type);
      }

      marker.id = _detected_ids[i];

      geometry_msgs::msg::PoseStamped tmpPoseIn, tmpPoseOut;
      tmpPoseIn.header = marker.header;
      tmpPoseIn.pose = marker.pose;

      tf2::doTransform<geometry_msgs::msg::PoseStamped>(tmpPoseIn, tmpPoseOut, assurancetourix_to_map_transformation);

      if (side.compare("yellow") == 0) {
        tmpPoseOut.pose.position.x += 0.28;
      }

      tmpPoseOut.pose.position.z = 0;
      transformed_marker = marker;
      transformed_marker.header = tmpPoseOut.header;
      transformed_marker.pose = tmpPoseOut.pose;

      if ((side.compare("blue") == 0) && (marker.id <= 10) && (6 <= marker.id)) {
        marker_array_ennemies.markers.push_back(transformed_marker);
      } else if ((side.compare("yellow") == 0) && (marker.id <= 5) && (1 <= marker.id)) {
        marker_array_ennemies.markers.push_back(transformed_marker);
      }

      if ((side.compare("yellow") == 0) && (marker.id <= 10) && (6 <= marker.id)) {
        marker_array_allies.markers.push_back(transformed_marker);
      } else if ((side.compare("blue") == 0) && (marker.id <= 5) && (1 <= marker.id)) {
        marker_array_allies.markers.push_back(transformed_marker);
      }
void Assurancetourix::estimate_arucos_poses() {

      marker_pub_->publish(marker);
    }
    transformed_marker_pub_ennemies_->publish(marker_array_ennemies);
    transformed_marker_pub_allies_->publish(marker_array_allies);
  }
}

#ifdef SIMULATION
rclcpp::Time Assurancetourix::get_sim_time(std::shared_ptr<webots::Robot> wb_robot) {
  double seconds = 0;
  double nanosec = modf(wb_robot->getTime(), &seconds) * 1e9;
  return rclcpp::Time((uint32_t)seconds, (uint32_t)nanosec);
}
#endif /* SIMULATION */

Assurancetourix::~Assurancetourix() {
#ifdef CAMERA
  _cap.release();
#endif // CAMERA
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
