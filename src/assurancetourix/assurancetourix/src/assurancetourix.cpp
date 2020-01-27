#include <assurancetourix.hpp>


Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  #ifdef MIPI_CAMERA
  arducam::arducam_init_camera(&camera_instance);
  arducam::arducam_set_resolution(camera_instance, &width, &height);
  arducam::arducam_software_auto_exposure(camera_instance, 1);
  arducam::arducam_software_auto_white_balance(camera_instance, 1);
  usleep(1000 * 1000 * 1);
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


cv::Mat * Assurancetourix::get_image(arducam::CAMERA_INSTANCE camera_instance, int width, int height) {
    arducam::IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    arducam::BUFFER *buffer = arducam::arducam_capture(camera_instance, &fmt, 3000);
    if (!buffer)
        return NULL;
    width = VCOS_ALIGN_UP(width, 32);
    height = VCOS_ALIGN_UP(height, 16);
    cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
    arducam::arducam_release_buffer(buffer);
    return image;
}


void Assurancetourix::init_parameters() {
  cv_img_bridge.encoding = "bgr8";
  cv_img_bridge.header.frame_id = "map";

  marker.header.frame_id = "map";
  marker.header.stamp.sec = 10;
  marker.header.stamp.nanosec = 100000000;
  marker.ns = "rviz";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 250.0;
  marker.color.b = 0.0;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 100000000;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;
  marker.id = 0;

  useless_point.x = 1;
  useless_point.y = 1;
  useless_point.z = 1;
  useless_point_vector.push_back(useless_point);
  marker.points = useless_point_vector;

  useless_color.a = 1.0;
  useless_color.r = 250.0;
  useless_color.g = 0.0;
  useless_color.b = 0.0;
  useless_color_vector.push_back(useless_color);
  marker.colors = useless_color_vector;

  marker.frame_locked = false;
  marker.text = "useless";
  marker.mesh_resource = "null";
  marker.mesh_use_embedded_materials = false;
}


void Assurancetourix::detect() {
  #ifdef MIPI_CAMERA
  _frame = get_image(camera_instance, width, height);
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

      marker.pose.position.x = _tvecs[i].operator[](0);
      marker.pose.position.y = _tvecs[i].operator[](1);
      marker.pose.position.z = _tvecs[i].operator[](2);

      tf2::Quaternion q;
      q.setRPY(_rvecs[i].operator[](0), _rvecs[i].operator[](1), _rvecs[i].operator[](2));

      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      marker.id = _detected_ids[i];

      marker_pub_->publish(marker);
      RCLCPP_INFO(this->get_logger(), "id: %d", _detected_ids[i]);
    }
  }
}


Assurancetourix::~Assurancetourix() {
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
