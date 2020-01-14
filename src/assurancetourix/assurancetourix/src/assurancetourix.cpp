#include <assurancetourix.hpp>


Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
  }
  exit(-1);
}


Assurancetourix::~Assurancetourix() {
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
