#include "nav2_gradient_costmap_plugin/gradient_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_gradient_costmap_plugin
{

GradientLayer::GradientLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{

}

void GradientLayer::marker_callback(const std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
{
  coord_from_frame.clear();
  if (int(msg->markers.size()) == 0)
  {
    return;
  }

  for(int i=0; i < int(msg->markers.size()); i++){
    std::array<double, 2> tmp_coord{{msg->markers[i].pose.position.x, msg->markers[i].pose.position.y}};
    coord_from_frame.push_back(tmp_coord);
  }
}
// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
GradientLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("gradient_size", rclcpp::ParameterValue(5));
  node_->get_parameter(name_ + "." + "gradient_size", GRADIENT_SIZE);
  declareParameter("gradient_factor", rclcpp::ParameterValue(60));
  node_->get_parameter(name_ + "." + "gradient_factor", GRADIENT_FACTOR);
  declareParameter("markers_topic", rclcpp::ParameterValue("coordonate_position_transform"));
  node_->get_parameter(name_ + "." + "markers_topic", topic);

  need_recalculation_ = false;
  coord_from_frame.clear();
  marker_subscriber = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      topic, qos, std::bind(&GradientLayer::marker_callback, this, std::placeholders::_1));
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
GradientLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_ || coord_from_frame.empty()) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  std::vector<std::array<double, 2>> coord_from_frame_at_t = coord_from_frame;
  std::vector<std::array<double, 2>> coord_from_index;
  for (std::array<double, 2> coord: coord_from_frame_at_t){
    std::array<double, 2> tmp;
    tmp[0] = coord[0]*50+101; //50: celulles by meters, 100: offset to be on the map
    tmp[1] = coord[1]*50+101; //50: celulles by meters, 100: offset to be on the map
    coord_from_index.push_back(tmp);
  }

  for (std::array<double, 2> cost: coord_from_index){
    for (int gradient_layer = 0; gradient_layer<LETHAL_OBSTACLE/GRADIENT_FACTOR; gradient_layer++){
      int gradient_cost = LETHAL_OBSTACLE - gradient_layer*GRADIENT_FACTOR;
      int gradient_radius = GRADIENT_SIZE*(gradient_layer+1);
      double max_rect_i = cost[0]+gradient_radius;
      double max_rect_j = cost[1]+gradient_radius;
      double min_rect_i = cost[0]-gradient_radius;
      double min_rect_j = cost[1]-gradient_radius;
      for (double k = min_rect_i; k<max_rect_i; k++){
        for (double v = min_rect_j; v<max_rect_j; v++){
          unsigned char old_cost = master_array[master_grid.getIndex(k, v)];
          if ( (k-cost[0])*(k-cost[0])+(v-cost[1])*(v-cost[1])<=gradient_radius*gradient_radius
                && (old_cost == NO_INFORMATION || old_cost <= gradient_cost) ){
            master_array[master_grid.getIndex(k, v)] = gradient_cost;
          }
        }
      }
    }
  }
}

}  // namespace nav2_gradient_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
