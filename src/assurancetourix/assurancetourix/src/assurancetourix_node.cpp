#include <assurancetourix.hpp>


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto assurancetourix_node = std::make_shared<Assurancetourix>();

  rclcpp::spin(assurancetourix_node);
  rclcpp::shutdown();
  return 0;
}
