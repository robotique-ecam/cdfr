#include <assurancetourix.hpp>


Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  #ifdef MIPI_CAMERA

    if (mode == 0) {int width = 3280, height = 2464;}
    else if (mode == 1) {int width = 2592, height = 1944;}
    else if (mode == 2) {int width = 1920, height = 1080;}
    else if (mode == 3) {int width = 1640, height = 1232;}
    else if (mode == 4) {int width = 1640, height = 922;}
    else if (mode == 5) {int width = 1280, height = 720;}
    else if (mode == 6) {int width = 640, height = 480;}

    else {
      RCLCPP_ERROR(this->get_logger(), "Invalid camera.mode in assurancetourix.yml, choose an integer between 0 and 6, current mode: %d", mode);
      exit(-1);
    }

    arducam::arducam_init_camera(&camera_instance);
    arducam::arducam_set_mode(camera_instance, 0); // width: 3280, height: 2464, pixelformat: pBAA, desc: (null)
    arducam::arducam_reset_control(camera_instance, 0x00980911);
    arducam::arducam_set_control(camera_instance, 0x00980911, exposure);
    arducam::arducam_software_auto_white_balance(camera_instance, 1);
    arducam::arducam_manual_set_awb_compensation( rgain, bgain);

  #else
    _cap.open(_api_id + _camera_id);
    if (!_cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
      exit(-1);
    }
  #endif //MIPI_CAMERA

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_aruco_position", qos);
  cv::namedWindow("anotated", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
  timer_ = this->create_wall_timer(0.05s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");

}


#ifdef MIPI_CAMERA // be sure the function is outside "if"
void Assurancetourix::get_image() {
    arducam::IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    arducam::BUFFER *buffer = arducam::arducam_capture(camera_instance, &fmt, 3000);
    if (buffer) {
      width = VCOS_ALIGN_UP(width, 32);
      height = VCOS_ALIGN_UP(height, 16);
      cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
      cv::cvtColor(*image, *image, cv::COLOR_YUV2RGB_I420);
      arducam::arducam_release_buffer(buffer);
      cv::flip(*image, *image, -1);
      _frame = *image;
      delete image;
    }
}
#endif //MIPI_CAMERA


void Assurancetourix::init_parameters() {

  this->declare_parameter("image.show_image");
  this->get_parameter_or<bool>("image.show_image", show_image, true);

  #ifdef MIPI_CAMERA
    this->declare_parameter("camera.exposure");
    this->declare_parameter("camera.rgain");
    this->declare_parameter("camera.bgain");
    this->declare_parameter("camera.mode");
    this->get_parameter_or<int>("camera.exposure", exposure, 250);
    this->get_parameter_or<uint>("camera.rgain", rgain, 50);
    this->get_parameter_or<uint>("camera.bgain", bgain, 280);
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

  this->get_parameter_or<double>("image.contrast", contrast, 2.0);
  this->get_parameter_or<std::vector<double>>("rviz_settings.blue_color_ArUco", blue_color_ArUco, {0.0, 0.0, 255.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.yellow_color_ArUco", yellow_color_ArUco, {255.0, 255.0, 0.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.default_color_ArUco", default_color_ArUco, {120.0, 120.0, 120.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.arrow_scale", arrow_scale, {0.3, 0.05, 0.05});
  this->get_parameter_or<std::vector<double>>("rviz_settings.game_elements_scale", game_elements_scale, {0.07, 0.07, 0.01});
  this->get_parameter_or<uint>("rviz_settings.robot_type", robot_type, 0);
  this->get_parameter_or<uint>("rviz_settings.game_element_type", game_element_type, 1);
  this->get_parameter_or<int>("rviz_settings.lifetime_sec", lifetime_sec, 5);
  this->get_parameter_or<int>("rviz_settings.lifetime_nano_sec", lifetime_nano_sec, 0);
  this->get_parameter_or<String>("rviz_settings.header_frame_id", header_frame_id, "map");

  RCLCPP_INFO(this->get_logger(), "contrast: %f", contrast);

  // initialisation of marker message
  marker.header.frame_id = header_frame_id;
  marker.color.a = 1.0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime.sec = lifetime_sec;
  marker.lifetime.nanosec = lifetime_nano_sec;

  starter_flag = true;

}


void Assurancetourix::detect() {
  #ifdef MIPI_CAMERA
    get_image();
  #else
    _cap.read(_frame);
    //cv::cvtColor(_frame, _frame, cv::COLOR_RGB2GRAY);
  #endif //MIPI_CAMERA

  _frame.convertTo(raised_contrast, -1, contrast, 0);
  cv::cvtColor(raised_contrast, raised_contrast, cv::COLOR_RGB2GRAY);

  if (starter_flag) {
    image_minus_t = raised_contrast;
    starter_flag = false;
  }

  //bitwise_xor(raised_contrast, image_minus_t, xor_image);
  //bitwise_not(xor_image, xor_image);
  //cv::GaussianBlur(raised_contrast,raised_contrast,Size(3,3),0);
  raised_contrast.copyTo(_anotated);
  //raised_contrast.copyTo(_anotated);
  marker.header.stamp = this->get_clock()->now();

  _detect_aruco(_anotated);
  _anotate_image(_anotated);

  if (show_image) {
    cv::imshow("anotated", _anotated);
    cv::imshow("origin", _frame);
    cv::waitKey(3);
  }
  image_minus_t = raised_contrast;
}


void Assurancetourix::_detect_aruco(Mat img) {
  cv::aruco::detectMarkers(img, _dictionary, _marker_corners, _detected_ids, _parameters, _rejected_candidates);
  if (show_image) {
    //drawDetectedMarkers on the image (activate only for debug)
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

    for(int i=0; i<int(_detected_ids.size()); i++) {

      if (show_image) {
        //drawAxis on the image (activate only for debug)
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

      if ( (_detected_ids[i] <= 5) && (1 <= _detected_ids[i]) ){
        set_vision_for_rviz(blue_color_ArUco, arrow_scale, robot_type);
      }
      else if ( (_detected_ids[i] <= 10) && (6 <= _detected_ids[i]) ){
        set_vision_for_rviz(yellow_color_ArUco, arrow_scale, robot_type);
      }
      else {set_vision_for_rviz(default_color_ArUco, game_elements_scale, game_element_type);
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
  #endif //MIPI_CAMERA
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
