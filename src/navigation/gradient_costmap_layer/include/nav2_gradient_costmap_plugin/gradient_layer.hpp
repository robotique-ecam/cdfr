#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav2_costmap_2d/costmap_layer.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace nav2_gradient_costmap_plugin {

class GradientLayer : public nav2_costmap_2d::Layer {
public:
  GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void reset() { return; }

  virtual void onFootprintChanged();

private:
  void marker_callback(const std::shared_ptr<visualization_msgs::msg::MarkerArray> msg);
  void ally_odom_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);

  std::string topic = "coordonate_position_transform";
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ally_odom_subscriber;
  visualization_msgs::msg::Marker received_marker;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  std::vector<std::array<double, 2>> coord_from_frame;
  std::vector<bool> predictedIndexes;
  geometry_msgs::msg::Pose ally_pose;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR;
};

} // namespace nav2_gradient_costmap_plugin

#endif // GRADIENT_LAYER_HPP_
