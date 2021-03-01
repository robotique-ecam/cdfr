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

    /**
     * @brief Constructor
     */
    CostmapToPolygonsDBSMCCH();

    /**
     * @brief Destructor
     */
    virtual ~CostmapToPolygonsDBSMCCH();

    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(rclcpp::Node::SharedPtr nh) override;

    virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap){};

    virtual void updateCostmap2D(){};

    virtual void compute(){};
    /**
     * @brief Get a shared instance of the current polygon container
     * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
     * @return Shared instance of the current polygon container
     */
    PolygonContainerConstPtr getPolygons();


  protected:

   /**
    * @brief Thread-safe update of the internal polygon container (that is shared using getPolygons() from outside this class)
    * @param polygons Updated polygon container
    */
   void updatePolygonContainer(PolygonContainerPtr polygons);

  private:


    PolygonContainerPtr polygons_; //!< Current shared container of polygons
    std::mutex mutex_; //!< Mutex that keeps track about the ownership of the shared polygon instance

    void setPolygon(std::vector<std::vector<float>> array);

    bool insideTheBoard(std::vector<float> arr);

};


} //end namespace teb_local_planner

#endif /* COSTMAP_TO_POLYGONS_H_ */
