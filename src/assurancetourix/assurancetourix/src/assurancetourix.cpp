#include <assurancetourix.hpp>


Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
    exit(-1);
  }

  image_pub_ = image_transport::create_publisher(this, "detected_aruco", rmw_qos_profile_default);

  timer_ = this->create_wall_timer(1s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
}


void Assurancetourix::init_parameters() {
  cv_img_bridge.encoding = "bgr8";
  cv_img_bridge.header.frame_id = "robotbase";
}


void Assurancetourix::detect() {

  _cap.read(_frame);
  _frame.copyTo(_anotated);
  cv_img_bridge.header.stamp = this->get_clock()->now();

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
  if (_detected_ids.size() > 0)
    cv::aruco::drawDetectedMarkers(img, _marker_corners, _detected_ids);

    cv::aruco::estimatePoseSingleMarkers(_marker_corners, 0.05, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);
    for(int i=0; i<_detected_ids.size(); i++)
      cv::aruco::drawAxis(img, _cameraMatrix, _distCoeffs, _rvecs[i], _tvecs[i], 0.1);

}


Assurancetourix::~Assurancetourix() {
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
