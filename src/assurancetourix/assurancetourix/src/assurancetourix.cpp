#include <assurancetourix.hpp>

Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

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

#else
  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
    exit(-1);
  }
#endif // MIPI_CAMERA

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_aruco_position", qos);
  if (show_image) {
    cv::namedWindow("anotated", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
  }
  timer_ = this->create_wall_timer(0.1s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
}

#ifdef MIPI_CAMERA
void Assurancetourix::get_image() {
  arducam::IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
  arducam::BUFFER *buffer = arducam::arducam_capture(camera_instance, &fmt, 3000);
  if (mode == 0) {
    int width = 3280, height = 2464;
  } else if (mode == 1) {
    int width = 2592, height = 1944;
  } else if (mode == 2) {
    int width = 1920, height = 1080;
  } else if (mode == 3) {
    int width = 1640, height = 1232;
  } else if (mode == 4) {
    int width = 1640, height = 922;
  } else if (mode == 5) {
    int width = 1280, height = 720;
  } else if (mode == 6) {
    int width = 640, height = 480;
  }
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
#endif // MIPI_CAMERA

void Assurancetourix::init_parameters() {

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
#endif

  // declare variables from yml
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

  this->get_parameter_or<double>("image.contrast", contrast, 3.0);
  this->get_parameter_or<std::vector<double>>("rviz_settings.blue_color_ArUco", blue_color_ArUco, {0.0, 0.0, 255.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.yellow_color_ArUco", yellow_color_ArUco, {255.0, 255.0, 0.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.default_color_ArUco", default_color_ArUco, {120.0, 120.0, 120.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.arrow_scale", arrow_scale, {0.3, 0.05, 0.05});
  this->get_parameter_or<std::vector<double>>("rviz_settings.game_elements_scale", game_elements_scale, {0.07, 0.07, 0.01});
  this->get_parameter_or<uint>("rviz_settings.robot_type", robot_type, 0);
  this->get_parameter_or<uint>("rviz_settings.game_element_type", game_element_type, 1);
  this->get_parameter_or<int>("rviz_settings.lifetime_sec", lifetime_sec, 5);
  this->get_parameter_or<int>("rviz_settings.lifetime_nano_sec", lifetime_nano_sec, 0);
  this->get_parameter_or<String>("rviz_settings.header_frame_id", header_frame_id, "assurancetourix/map");

  RCLCPP_INFO(this->get_logger(), "contrast: %f", contrast);

  // initialisation of marker message
  marker.header.frame_id = header_frame_id;
  marker.color.a = 1.0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime.sec = lifetime_sec;
  marker.lifetime.nanosec = lifetime_nano_sec;
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
  _cap.read(_frame);
#endif // MIPI_CAMERA
  auto end_camera = std::chrono::high_resolution_clock::now();

  cv::resize(_frame, _frame, Size(), 0.5, 0.5, cv::INTER_LINEAR);
  _frame.convertTo(raised_contrast, -1, contrast, 0);

  raised_contrast.copyTo(_anotated);

  marker.header.stamp = this->get_clock()->now();

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
  // image_minus_t = raised_contrast;
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
}

void Assurancetourix::_anotate_image(Mat img) {
  if (_detected_ids.size() > 0) {

    cv::aruco::estimatePoseSingleMarkers(_marker_corners, 0.06, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);

    for (int i = 0; i < int(_detected_ids.size()); i++) {

      if (show_image) {
        // drawAxis on the image (activate only for debug)
        cv::aruco::drawAxis(img, _cameraMatrix, _distCoeffs, _rvecs[i], _tvecs[i], 0.1);
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
