#include <assurancetourix.hpp>


Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  #ifdef MIPI_CAMERA
  arducam::arducam_init_camera(&camera_instance);
  arducam::arducam_set_mode(camera_instance, 2); // width: 1920, height: 1080, pixelformat: pBAA, desc: (null)
  arducam::arducam_software_auto_white_balance(camera_instance, 1);
  arducam::arducam_software_auto_exposure(camera_instance, 1);
  // rclcpp::sleep_for(1s);
  #else // Standard camera
  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
    exit(-1);
  }
  #endif // MIPI_CAMERA

  image_pub_ = image_transport::create_publisher(this, "detected_aruco", rmw_qos_profile_default);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_aruco_position", qos);

  timer_ = this->create_wall_timer(0.1s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");

}


#ifdef MIPI_CAMERA
void Assurancetourix::get_image() {
    arducam::IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    arducam::BUFFER *buffer = arducam::arducam_capture(camera_instance, &fmt, 3000);
    if (buffer) {
      width = VCOS_ALIGN_UP(width, 32);
      height = VCOS_ALIGN_UP(height, 16);
      cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
      cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
      arducam::arducam_release_buffer(buffer);
      _frame = *image;
      delete image;
    }
}
#endif // MIPI_CAMERA


void Assurancetourix::init_parameters() {
  cv_img_bridge.encoding = "bgr8";
  cv_img_bridge.header.frame_id = "map";

  //colors of the ArUco
  blue_color_ArUco.r = 0.0;
  blue_color_ArUco.g = 0.0;
  blue_color_ArUco.b = 255.0;
  blue_color_ArUco.a = 1.0;

  yellow_color_ArUco.r = 255.0;
  yellow_color_ArUco.g = 255.0;
  yellow_color_ArUco.b = 0.0;
  yellow_color_ArUco.a = 1.0;

  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "rviz";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.07;
  marker.scale.y = 0.07;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 250.0;
  marker.color.b = 0.0;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 500000000;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;
  marker.id = 0;

  useless_point.x = 0;
  useless_point.y = 1;
  useless_point.z = 0;
  useless_point_vector.push_back(useless_point);
  marker.points = useless_point_vector;

  default_color_ArUco.a = 1.0;
  default_color_ArUco.r = 255.0;
  default_color_ArUco.g = 255.0;
  default_color_ArUco.b = 255.0;
  useless_color_vector.push_back(default_color_ArUco);
  marker.colors = useless_color_vector;

  marker.frame_locked = false;
  marker.text = "useless";
  marker.mesh_resource = "null";
  marker.mesh_use_embedded_materials = false;
}


void Assurancetourix::detect() {
  #ifdef MIPI_CAMERA
  get_image();
  #else
  _cap.read(_frame);
  #endif // MIPI_CAMERA
  _frame.copyTo(_anotated);
  cv_img_bridge.header.stamp = this->get_clock()->now();
  marker.header.stamp = this->get_clock()->now();


  _detect_aruco(_anotated);
  _anotate_image(_anotated);


  cv_img_bridge.image = _anotated;
  cv_img_bridge.toImageMsg(img_msg);


  image_pub_.publish(img_msg);
}


void Assurancetourix::_detect_aruco(Mat img) {
  cv::aruco::detectMarkers(img, _dictionary, _marker_corners, _detected_ids, _parameters, _rejected_candidates);
}


void Assurancetourix::_anotate_image(Mat img) {
  if (_detected_ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(img, _marker_corners, _detected_ids);
    cv::aruco::estimatePoseSingleMarkers(_marker_corners, 0.05, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);


    for(int i=0; i<int(_detected_ids.size()); i++) {

      cv::aruco::drawAxis(img, _cameraMatrix, _distCoeffs, _rvecs[i], _tvecs[i], 0.1);

      marker.pose.position.x = 3*_tvecs[i].operator[](0);
      marker.pose.position.y = 3*_tvecs[i].operator[](1);
      marker.pose.position.z = 3*_tvecs[i].operator[](2);

      double angle = norm(_rvecs[i]);
      Vec3d axis = _rvecs[i] / angle;

      tf2::Quaternion q;
      q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      if ( (_detected_ids[i] <= 5) && (1 <= _detected_ids[i]) ){
        marker.color = blue_color_ArUco;
      }
      else if ( (_detected_ids[i] <= 10) && (6 <= _detected_ids[i]) ){
        marker.color = yellow_color_ArUco;
      }
      else {marker.color = default_color_ArUco;}
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
