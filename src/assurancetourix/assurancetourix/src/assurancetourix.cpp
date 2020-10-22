#include <assurancetourix.hpp>

Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  auto on_parameter_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {

      for (auto & new_parameter : event->new_parameters) {
        if (new_parameter.name == "side" && (new_parameter.value.string_value == "blue" || new_parameter.value.string_value == "yellow")){
          RCLCPP_INFO(this->get_logger(), "new_parameter from node %s, side = %s \n", event->node.c_str(), new_parameter.value.string_value.c_str());
          side = new_parameter.value.string_value;
        }
      }

      for (auto & changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "side" && (changed_parameter.value.string_value == "blue" || changed_parameter.value.string_value == "yellow")){
          RCLCPP_INFO(this->get_logger(), "changed_parameter from node %s, side = %s \n", event->node.c_str(), changed_parameter.value.string_value.c_str());
          side = changed_parameter.value.string_value;
        }
      }
    };

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

  transformClient = this->create_client<transformix_msgs::srv::TransformixParametersTransformStamped>("transformix/get_transform");
  getTransformation(assurancetourix_to_map_transformation);


#ifdef MIPI_CAMERA

  RCLCPP_INFO(this->get_logger(), "define MIPI_CAMERA ");
  if ((mode > 6) || (mode < 0)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid camera.mode in assurancetourix.yml, choose an integer between 0 and 6, current mode: %d", mode);
    exit(-1);
  }

  arducam::arducam_init_camera(&camera_instance);
  arducam::arducam_set_mode(camera_instance, mode);
  arducam::arducam_reset_control(camera_instance, 0x00980911);
  arducam::arducam_set_control(camera_instance, 0x00980911, exposure);
  arducam::arducam_software_auto_white_balance(camera_instance, 1);
  arducam::arducam_manual_set_awb_compensation(rgain, bgain);

  timer_ = NULL;

  _enable_aruco_detection = this->create_service<std_srvs::srv::SetBool>("enable_aruco_detection", std::bind(&Assurancetourix::handle_aruco_detection_enable, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

#endif // MIPI_CAMERA

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_aruco_position", qos);
  transformed_marker_pub_ennemies_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_for_gradient_layer, qos);
  transformed_marker_pub_allies_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(allies_positions_topic, qos);

  if (show_image) {
    cv::namedWindow("anotated", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
  }

#ifdef SIMULATION
  RCLCPP_INFO(this->get_logger(), "SIMULATION defined");
  RCLCPP_INFO(this->get_logger(), "Simulated robots:");
  for (auto robot : robots) {
    RCLCPP_INFO(this->get_logger(), "%s", robot.c_str());
  }

  char mypath[] = "WEBOTS_ROBOT_NAME=game_manager";
  putenv(mypath);

  wb_supervisor = std::make_shared<webots::Supervisor>();

  timer_ = this->create_wall_timer(std::chrono::seconds(1 / refresh_frequency), std::bind(&Assurancetourix::simulation_marker_callback, this));
#endif
  timer_ = this->create_wall_timer(0.3s, std::bind(&Assurancetourix::detect, this)); //to remove for PR
  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
}

#ifdef MIPI_CAMERA
// image capture trough mipi_camera
void Assurancetourix::get_image() {
  arducam::IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
  arducam::BUFFER *buffer = arducam::arducam_capture(camera_instance, &fmt, 3000);
  if (buffer) {
    width = VCOS_ALIGN_UP(3280, 32);
    height = VCOS_ALIGN_UP(2464, 16);
    cv::Mat *image = new cv::Mat(cv::Size(width, (int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2GRAY_I420);
    arducam::arducam_release_buffer(buffer);
    cv::flip(*image, *image, -1);
    _frame = *image;
    delete image;
  }
}

//service enabling/disabling aruco detection, enabled by default
//service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
void Assurancetourix::handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                  const std_srvs::srv::SetBool::Response::SharedPtr response) {

  if (request->data) {
    timer_ = this->create_wall_timer(0.3s, std::bind(&Assurancetourix::detect, this));
    response->message = "Aruco detection is enabled";
    RCLCPP_WARN(this->get_logger(), "Aruco detection is enabled");
  } else {
    timer_ = NULL;
    response->message = "Aruco detection is disabled";
    RCLCPP_WARN(this->get_logger(), "Aruco detection is disabled");
  }
  response->success = true;
}
#endif // MIPI_CAMERA

#ifdef SIMULATION
// webots element positioning
void Assurancetourix::simulation_marker_callback() {
  visualization_msgs::msg::MarkerArray marker_array;
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

    x = wb_supervisor->getFromDef(robot)->getPosition()[0];
    y = 2 - wb_supervisor->getFromDef(robot)->getPosition()[2];

    webots_marker.header.stamp = this->get_clock()->now();
    webots_marker.pose.position.x = x;
    webots_marker.pose.position.y = y;
    webots_marker.text = robot;
    webots_marker.id = id;

    marker_array.markers.push_back(webots_marker);
    id++;
  }

  transformed_marker_pub_ennemies_->publish(marker_array);
}
#endif // SIMULATION

void Assurancetourix::init_parameters() {

  // declare variables from yml
  this->declare_parameter("image.show_image");
  this->get_parameter_or<bool>("image.show_image", show_image, false);

#ifdef MIPI_CAMERA
  this->declare_parameter("camera.exposure");
  this->declare_parameter("camera.rgain");
  this->declare_parameter("camera.bgain");
  this->declare_parameter("camera.mode");
  this->get_parameter_or<int>("camera.exposure", exposure, 250);
  this->get_parameter_or<uint>("camera.rgain", rgain, 3110);
  this->get_parameter_or<uint>("camera.bgain", bgain, 5160);
  this->get_parameter_or<int>("camera.mode", mode, 0);
#endif // MIPI_CAMERA

#ifdef SIMULATION
  this->declare_parameter("simulation.robots");
  this->declare_parameter("simulation.refresh_frequency");
  robots = this->get_parameter("simulation.robots").as_string_array();
  this->get_parameter_or<int>("simulation.refresh_frequency", refresh_frequency, 5);
#endif // SIMULATION

  this->declare_parameter("image.contrast");
  this->declare_parameter("rviz_settings.blue_color_ArUco");
  this->declare_parameter("rviz_settings.yellow_color_ArUco");
  this->declare_parameter("rviz_settings.default_color_ArUco");
  this->declare_parameter("rviz_settings.arrow_scale");
  this->declare_parameter("rviz_settings.game_elements_scale");
  this->declare_parameter("rviz_settings.robot_type");
  this->declare_parameter("rviz_settings.game_element_type");
  this->declare_parameter("rviz_settings.lifetime_sec");
  this->declare_parameter("rviz_settings.lifetime_nano_sec");
  this->declare_parameter("rviz_settings.header_frame_id");
  this->declare_parameter("topic_for_gradient_layer");
  this->declare_parameter("topic_for_allies_position");
  this->declare_parameter("side");

  this->get_parameter_or<double>("image.contrast", contrast, 1.2);
  this->get_parameter_or<std::vector<double>>("rviz_settings.blue_color_ArUco", blue_color_ArUco, {0.0, 0.0, 255.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.yellow_color_ArUco", yellow_color_ArUco, {255.0, 255.0, 0.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.default_color_ArUco", default_color_ArUco, {120.0, 120.0, 120.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.arrow_scale", arrow_scale, {0.05, 0.05, 0.05});
  this->get_parameter_or<std::vector<double>>("rviz_settings.game_elements_scale", game_elements_scale, {0.07, 0.07, 0.01});
  this->get_parameter_or<uint>("rviz_settings.robot_type", robot_type, 2);
  this->get_parameter_or<uint>("rviz_settings.game_element_type", game_element_type, 1);
  this->get_parameter_or<int>("rviz_settings.lifetime_sec", lifetime_sec, 2);
  this->get_parameter_or<int>("rviz_settings.lifetime_nano_sec", lifetime_nano_sec, 0);
  this->get_parameter_or<std::string>("rviz_settings.header_frame_id", header_frame_id, "assurancetourix");
  this->get_parameter_or<std::string>("topic_for_gradient_layer", topic_for_gradient_layer, "/default_topic_for_gradient_layer");
  this->get_parameter_or<std::string>("topic_for_allies_position", allies_positions_topic, "/default_topic_for_allies_positions");
  this->get_parameter_or<std::string>("side", side, "blue");

  // initialisation of markers
  marker.header.frame_id = header_frame_id;
  marker.color.a = 1.0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime.sec = lifetime_sec;
  marker.lifetime.nanosec = lifetime_nano_sec;

  base_frame = "map";
  transformed_marker.header.frame_id = base_frame;
  transformed_marker.color.a = 1.0;
  transformed_marker.action = visualization_msgs::msg::Marker::ADD;
  transformed_marker.lifetime.sec = lifetime_sec;
  transformed_marker.lifetime.nanosec = lifetime_nano_sec;
}

void Assurancetourix::detect() {

#ifdef EXTRALOG //if enabled show the time elapsed to do some actions (capture the image, find the coordonates of the arucos in it and the total)
//will also show a marker at the assurancetourix origin frame position
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

  auto start_total = std::chrono::high_resolution_clock::now();
  auto start_camera = std::chrono::high_resolution_clock::now();
  std::ios_base::sync_with_stdio(false);
#endif

#ifdef MIPI_CAMERA
  get_image();
#else
  // read an image from an image file
  _frame = imread("/home/phileas/covid_home.jpg", IMREAD_GRAYSCALE);
#endif // MIPI_CAMERA

#ifdef EXTRALOG
  auto end_camera = std::chrono::high_resolution_clock::now();
#endif

  //cv::resize(_frame, _frame, Size(), 0.5, 0.5, cv::INTER_LINEAR);
  _frame.convertTo(raised_contrast, -1, contrast, 0);

  raised_contrast.copyTo(_anotated);

#ifdef EXTRALOG
  marker.header.stamp = this->get_clock()->now();
  transformed_marker.header.stamp = this->get_clock()->now();

  marker_pub_->publish(marker);

  auto start_detection = std::chrono::high_resolution_clock::now();
#endif

  _detect_aruco(_anotated);
  _anotate_image(_anotated);

#ifdef EXTRALOG
  auto end_detection = std::chrono::high_resolution_clock::now();
  auto end_total = std::chrono::high_resolution_clock::now();


  double time_taken_camera = std::chrono::duration_cast<std::chrono::nanoseconds>(end_camera - start_camera).count();
  double time_taken_detection = std::chrono::duration_cast<std::chrono::nanoseconds>(end_detection - start_detection).count();
  double time_taken_total = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - start_total).count();

  time_taken_camera *= 1e-9;
  time_taken_detection *= 1e-9;
  time_taken_total *= 1e-9;

  RCLCPP_INFO(this->get_logger(), "time of camera: %f", time_taken_camera);
  RCLCPP_INFO(this->get_logger(), "time of detection: %f", time_taken_detection);
  RCLCPP_INFO(this->get_logger(), "time of total: %f", time_taken_total);
#endif

  if (show_image) {
    cv::imshow("anotated", _anotated);
    // cv::imshow("origin", _frame);
    cv::waitKey(3);
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
  RCLCPP_INFO(this->get_logger(), "Getting transform...");

  auto result = transformClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    transformation = result.get()->transform_stamped;
    RCLCPP_INFO(this->get_logger(), "Get transform");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transformation");
  }
}

void Assurancetourix::_detect_aruco(Mat img) {
  cv::aruco::detectMarkers(img, _dictionary, _marker_corners, _detected_ids, _parameters, _rejected_candidates);
  if (show_image) {
    // drawDetectedMarkers on the image (activate only for debug)
    cv::aruco::drawDetectedMarkers(img, _marker_corners, _detected_ids);
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

    cv::aruco::estimatePoseSingleMarkers(_marker_corners, 0.06, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);

    for (int i = 0; i < int(_detected_ids.size()); i++) {

      if (show_image) {
        // drawAxis on the image (activate only for debug)
        cv::aruco::drawAxis(img, _cameraMatrix, _distCoeffs, _rvecs[i], _tvecs[i], 0.1);
      }

      double x, y, z;
      x = _tvecs[i].operator[](0);
      y = _tvecs[i].operator[](1);
      z = _tvecs[i].operator[](2);

      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;

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

      float colorSideCoeff = 0;
      if (side.compare("yellow") == 0){
        colorSideCoeff = 0.2;
      }

      tmpPoseOut.pose.position.x += 0.15 + colorSideCoeff;
      tmpPoseOut.pose.position.z = 0;
      transformed_marker = marker;
      transformed_marker.header = tmpPoseOut.header;
      transformed_marker.pose = tmpPoseOut.pose;

      if ( (side.compare("blue")==0) && (marker.id <= 10) && (6 <= marker.id) ){
        marker_array_ennemies.markers.push_back(transformed_marker);
      }
      else if ( (side.compare("yellow")==0) && (marker.id <= 5) && (1 <= marker.id) ){
        marker_array_ennemies.markers.push_back(transformed_marker);
      }

      if ( (side.compare("yellow")==0) && (marker.id <= 10) && (6 <= marker.id) ){
        marker_array_allies.markers.push_back(transformed_marker);
      }
      else if ( (side.compare("blue")==0) && (marker.id <= 5) && (1 <= marker.id) ){
        marker_array_allies.markers.push_back(transformed_marker);
      }

      marker_pub_->publish(marker);
    }
    transformed_marker_pub_ennemies_->publish(marker_array_ennemies);
    transformed_marker_pub_allies_->publish(marker_array_allies);
  }
}

Assurancetourix::~Assurancetourix() {
#ifdef MIPI_CAMERA
  arducam::arducam_close_camera(camera_instance);
#endif // MIPI_CAMERA
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
