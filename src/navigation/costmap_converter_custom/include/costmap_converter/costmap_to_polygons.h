#ifndef COSTMAP_TO_POLYGONS_H_
#define COSTMAP_TO_POLYGONS_H_

#include <rclcpp/rclcpp.hpp>
#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <vector>
#include <algorithm>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <nav2_costmap_2d/array_parser.hpp>


namespace costmap_converter
{

class CostmapToPolygonsDBSMCCH : public BaseCostmapToPolygons
{
  public:

    CostmapToPolygonsDBSMCCH();

    virtual ~CostmapToPolygonsDBSMCCH();

    virtual void initialize(rclcpp::Node::SharedPtr nh) override;

    virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap){};

    virtual void updateCostmap2D(){};

    virtual void compute(){};

    PolygonContainerConstPtr getPolygons();


  protected:

   void updatePolygonContainer(PolygonContainerPtr polygons);


  private:

    PolygonContainerPtr polygons_;

    void setPolygon(std::vector<std::vector<float>> array);

    bool insideTheBoard(std::vector<float> arr);

};


} //end namespace teb_local_planner

#endif /* COSTMAP_TO_POLYGONS_H_ */
