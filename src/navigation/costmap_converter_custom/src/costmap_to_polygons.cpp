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

void CostmapToPolygonsDBSMCCH::setPolygon(std::vector<std::vector<float>> array)
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
    RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "%f %f %f %f", coords_x1_y1_x2_y2[0], coords_x1_y1_x2_y2[1], coords_x1_y1_x2_y2[2], coords_x1_y1_x2_y2[3]);

    if (insideTheBoard(coords_x1_y1_x2_y2)){
      float min_x, min_y, max_x, max_y;
      if (coords_x1_y1_x2_y2[0] < coords_x1_y1_x2_y2[2]){
        min_x = coords_x1_y1_x2_y2[0];
        max_x = coords_x1_y1_x2_y2[2];
      }
      else
      {
        min_x = coords_x1_y1_x2_y2[2];
        max_x = coords_x1_y1_x2_y2[0];
      }
      if (coords_x1_y1_x2_y2[1] < coords_x1_y1_x2_y2[3]){
        min_y = coords_x1_y1_x2_y2[1];
        max_y = coords_x1_y1_x2_y2[3];
      }
      else
      {
        min_y = coords_x1_y1_x2_y2[3];
        max_y = coords_x1_y1_x2_y2[1];
      }
      if ((max_y - min_y) < (max_x - min_x)){
        for (int j = 0; j < (max_x - min_x)*100; j++){
          p.x = min_x + float(j)/100;
          p.y = min_y;
          poly.points.push_back(p);
          p.x = max_x + float(j)/100;
          p.y = min_y;
          poly.points.push_back(p);
          polygons->push_back(poly);
          poly.points.clear();
        }
      }
      else
      {
        for (int j = 0; j < (max_y - min_y)*100; j++){
          p.x = min_x;
          p.y = min_y + float(j)/100;
          poly.points.push_back(p);
          p.x = max_x;
          p.y = min_y + float(j)/100;
          poly.points.push_back(p);
          polygons->push_back(poly);
          poly.points.clear();
        }
      }
    }
    else
    {
      p.x = coords_x1_y1_x2_y2[0];
      p.y = coords_x1_y1_x2_y2[1];
      poly.points.push_back(p);
      p.x = coords_x1_y1_x2_y2[2];
      p.y = coords_x1_y1_x2_y2[3];
      poly.points.push_back(p);
      polygons->push_back(poly);
      poly.points.clear();
    }
  }
  updatePolygonContainer(polygons);
}

bool CostmapToPolygonsDBSMCCH::insideTheBoard(std::vector<float> arr){
  bool pt1 = arr[0]<3.0 && arr[0]>0.0 && arr[1]<2.0 && arr[1]>0.0,
       pt2 = arr[2]<3.0 && arr[2]>0.0 && arr[3]<2.0 && arr[3]>0.0;
  return pt1 || pt2;
}

void CostmapToPolygonsDBSMCCH::updatePolygonContainer(PolygonContainerPtr polygons)
{
  polygons_ = polygons;
}


PolygonContainerConstPtr CostmapToPolygonsDBSMCCH::getPolygons()
{
  return polygons_;
}

}
