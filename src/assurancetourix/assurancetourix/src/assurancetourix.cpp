#include <assurancetourix.hpp>


Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
    exit(-1);
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detected_aruco", qos);

  timer_ = this->create_wall_timer(1s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
}


void Assurancetourix::init_parameters() {

}


void Assurancetourix::detect() {

  std::cout << "Running detection\n";

  _cap.read(_frame);
  _frame.copyTo(_anotated);
  img_msg.header.stamp = this->get_clock()->now();

  std::cout << "Captured frame\n";

  _detect_aruco(_anotated);
  _anotate_image(_anotated);

  std::cout << "Detected\n";

  cv_img_bridge.image = _anotated;
  cv_img_bridge.toImageMsg(img_msg);
.
  std::cout << "Publishing\n";

  image_pub_->publish(img_msg);
}


void Assurancetourix::_detect_aruco(Mat img) {
  cv::aruco::detectMarkers(img, _dictionary, _marker_corners, _detected_ids, _parameters, _rejected_candidates);
}


void Assurancetourix::_anotate_image(Mat img) {
  if (_detected_ids.size() > 0)
    cv::aruco::drawDetectedMarkers(img, _marker_corners, _detected_ids);
}


Assurancetourix::~Assurancetourix() {
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
