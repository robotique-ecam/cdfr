#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/misc.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH() : BaseCostmapToPolygons() {}

CostmapToPolygonsDBSMCCH::~CostmapToPolygonsDBSMCCH() {}

void CostmapToPolygonsDBSMCCH::initialize(rclcpp::Node::SharedPtr nh)
{
    BaseCostmapToPolygons::initialize(nh);

    std::string static_map_polygons_string;
    nh->declare_parameter("static_map_lines", rclcpp::ParameterValue("NULL"));
    nh->get_parameter("static_map_lines", static_map_polygons_string);

    if (static_map_polygons_string.compare("NULL") != 0) {
      std::string error_return;
      std::vector<std::vector<float>> array;
      array = nav2_costmap_2d::parseVVF(static_map_polygons_string, error_return);
      setPolygon(array);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "No static map in yaml static_map_lines");
    }
}

void CostmapToPolygonsDBSMCCH::setPolygon(std::vector<std::vector<float>> array){
  PolygonContainerPtr polygons(new std::vector<geometry_msgs::msg::Polygon>());
  geometry_msgs::msg::Point32 p;
  p.z = 0.0;
  geometry_msgs::msg::Polygon poly;
  for (std::vector<float> coords_x1_y1_x2_y2 : array){
    if (coords_x1_y1_x2_y2.size() != 4){
      RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "Invalid static map in yaml static_map_lines, one size != 4");
      return;
    }
    RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "%f %f %f %f", coords_x1_y1_x2_y2[0], coords_x1_y1_x2_y2[1], coords_x1_y1_x2_y2[2], coords_x1_y1_x2_y2[3]);

      }
      {
      }
      }
      else
      {
      }
      }
      {
        }
    }
    {
    }
}

}

void CostmapToPolygonsDBSMCCH::updatePolygonContainer(PolygonContainerPtr polygons)
{
  std::lock_guard<std::mutex> lock(mutex_);
  polygons_ = polygons;
}


PolygonContainerConstPtr CostmapToPolygonsDBSMCCH::getPolygons()
{
  std::lock_guard<std::mutex> lock(mutex_);
  PolygonContainerConstPtr polygons = polygons_;
  return polygons;
}

}
