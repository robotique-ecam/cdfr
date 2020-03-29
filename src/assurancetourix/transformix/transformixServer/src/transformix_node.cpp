#include <transformix.hpp>


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto transformix_node = std::make_shared<Transformix>();
  
  rclcpp::spin(transformix_node);
  rclcpp::shutdown();
  return 0;
}
