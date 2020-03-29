#include <assurancetourix.hpp>

Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  transformClient = this->create_client<transformix_services::srv::TransformixParametersTransformStamped>("get_transform");

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
    arducam::arducam_manual_set_awb_compensation( rgain, bgain);

  /*#else

    _cap.open(_api_id + _camera_id);
    if (!_cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
      exit(-1);
    } */
  #endif //MIPI_CAMERA

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_aruco_position", qos);
  transformed_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("coordonate_position_transform", qos);

  if (show_image) {
    cv::namedWindow("anotated", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
  }
  timer_ = this->create_wall_timer(0.1s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
  RCLCPP_INFO(this->get_logger(), "contrast: %f", contrast);
}

#ifdef MIPI_CAMERA
void Assurancetourix::get_image() {
    arducam::IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    arducam::BUFFER *buffer = arducam::arducam_capture(camera_instance, &fmt, 3000);
    if (buffer) {
      width = VCOS_ALIGN_UP(3280, 32);
      height = VCOS_ALIGN_UP(2464, 16);
      cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
      cv::cvtColor(*image, *image, cv::COLOR_YUV2GRAY_I420);
      arducam::arducam_release_buffer(buffer);
      cv::flip(*image, *image, -1);
      _frame = *image;
      delete image;
    }
}
#endif // MIPI_CAMERA

void Assurancetourix::init_parameters() {

  this->declare_parameter("image.show_image");
  this->get_parameter_or<bool>("image.show_image", show_image, false);

  #ifdef MIPI_CAMERA
    this->declare_parameter("camera.exposure");
    this->declare_parameter("camera.rgain");
    this->declare_parameter("camera.bgain");
    this->declare_parameter("camera.mode");
    this->get_parameter_or<int>("camera.exposure", exposure,250);
    this->get_parameter_or<uint>("camera.rgain", rgain, 3110);
    this->get_parameter_or<uint>("camera.bgain", bgain, 5160);
    this->get_parameter_or<int>("camera.mode", mode, 0);
  #endif

  //declare variables from yml
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

  // initialisation of marker message
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

  #ifdef MIPI_CAMERA
    get_image();
  #else
    _frame = imread("/home/phileas/covid_home.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    //_cap.read(_frame);
  #endif //MIPI_CAMERA
  auto end_camera = std::chrono::high_resolution_clock::now();

  cv::resize(_frame, _frame, Size(), 0.5, 0.5, cv::INTER_LINEAR);
  _frame.convertTo(raised_contrast, -1, contrast, 0);

  raised_contrast.copyTo(_anotated);

  marker.header.stamp = this->get_clock()->now();
  transformed_marker.header.stamp = this->get_clock()->now();

  marker_pub_->publish(marker);

  auto start_detection = std::chrono::high_resolution_clock::now();

  _detect_aruco(_anotated);
  _anotate_image(_anotated);

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

  if (show_image) {
    cv::imshow("anotated", _anotated);
    // cv::imshow("origin", _frame);
    cv::waitKey(3);
  }

}

void Assurancetourix::getTransformation(geometry_msgs::msg::TransformStamped& transformation){
  auto request = std::make_shared<transformix_services::srv::TransformixParametersTransformStamped::Request>();

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

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
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

    cv::aruco::estimatePoseSingleMarkers(_marker_corners, 0.06, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);

    for (int i = 0; i < int(_detected_ids.size()); i++) {

      if (show_image) {
        // drawAxis on the image (activate only for debug)
        cv::aruco::drawAxis(img, _cameraMatrix, _distCoeffs, _rvecs[i], _tvecs[i], 0.1);
      }

      double x,y,z;
      x= _tvecs[i].operator[](0) - 0.15;
      y= _tvecs[i].operator[](1) + 0.15;
      z= _tvecs[i].operator[](2) - 0.15;

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

      tmpPoseOut.pose.position.z = 0;
      transformed_marker = marker;
      transformed_marker.header = tmpPoseOut.header;
      transformed_marker.pose = tmpPoseOut.pose;

      transformed_marker_pub_ -> publish(transformed_marker);
      marker_pub_->publish(marker);
      RCLCPP_INFO(this->get_logger(), "id: %d", _detected_ids[i]);
    }
  }
}

Assurancetourix::~Assurancetourix() {
#ifdef MIPI_CAMERA
  arducam::arducam_close_camera(camera_instance);
#endif // MIPI_CAMERA
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
