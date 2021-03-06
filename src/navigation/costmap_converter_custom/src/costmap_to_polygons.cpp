#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/misc.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToLines, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToLines::CostmapToLines() : BaseCostmapToPolygons() {}

CostmapToLines::~CostmapToLines() {}

void CostmapToLines::initialize(rclcpp::Node::SharedPtr nh)
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

void CostmapToLines::setPolygon(std::vector<std::vector<float>> array)
{
  PolygonContainerPtr polygons(new std::vector<geometry_msgs::msg::Polygon>());
  geometry_msgs::msg::Point32 p;
  p.z = 0.0;
  geometry_msgs::msg::Polygon poly;
  for (std::vector<float> coords_x1_y1_x2_y2 : array){
    if (coords_x1_y1_x2_y2.size() != 4){
      RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "Invalid static map in yaml static_map_lines, one size != 4");
      return;
    }

    p.x = coords_x1_y1_x2_y2[0];
    p.y = coords_x1_y1_x2_y2[1];
    poly.points.push_back(p);
    p.x = coords_x1_y1_x2_y2[2];
    p.y = coords_x1_y1_x2_y2[3];
    poly.points.push_back(p);
    polygons->push_back(poly);
    poly.points.clear();
  }
  updatePolygonContainer(polygons);
}

bool CostmapToLines::insideTheBoard(std::vector<float> arr){
  bool pt1 = arr[0]<3.0 && arr[0]>0.0 && arr[1]<2.0 && arr[1]>0.0,
       pt2 = arr[2]<3.0 && arr[2]>0.0 && arr[3]<2.0 && arr[3]>0.0;
  return pt1 || pt2;
}

void CostmapToLines::updatePolygonContainer(PolygonContainerPtr polygons)
{
  polygons_ = polygons;
}


PolygonContainerConstPtr CostmapToLines::getPolygons()
{
  return polygons_;
}

}
