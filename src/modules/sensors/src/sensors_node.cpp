#include "sensors.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto sensors_node = std::make_shared<Sensors>();

  RCLCPP_INFO(sensors_node->get_logger(), " !");

  rclcpp::spin(sensors_node);
  rclcpp::shutdown();
  return 0;
}
