#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/misc.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace
{

/**
 * @brief Douglas-Peucker Algorithm for fitting lines into ordered set of points
 *
 * Douglas-Peucker Algorithm, see https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
 *
 * @param begin iterator pointing to the begin of the range of points
 * @param end interator pointing to the end of the range of points
 * @param epsilon distance criteria for removing points if it is closer to the line segment than this
 * @param result the simplified polygon
 */
std::vector<geometry_msgs::msg::Point32> douglasPeucker(std::vector<geometry_msgs::msg::Point32>::iterator begin,
  std::vector<geometry_msgs::msg::Point32>::iterator end, double epsilon)
{
  if (std::distance(begin, end) <= 2)
  {
    return std::vector<geometry_msgs::msg::Point32>(begin, end);
  }

  // Find the point with the maximum distance from the line [begin, end)
  double dmax = std::numeric_limits<double>::lowest();
  std::vector<geometry_msgs::msg::Point32>::iterator max_dist_it;
  std::vector<geometry_msgs::msg::Point32>::iterator last = std::prev(end);
  for (auto it = std::next(begin); it != last; ++it)
  {
    double d = costmap_converter::computeSquaredDistanceToLineSegment(*it, *begin, *last);
    if (d > dmax)
    {
      max_dist_it = it;
      dmax = d;
    }
  }

  if (dmax < epsilon * epsilon)
  { // termination criterion reached, line is good enough
    std::vector<geometry_msgs::msg::Point32> result;
    result.push_back(*begin);
    result.push_back(*last);
    return result;
  }

  // Recursive calls for the two splitted parts
  auto firstLineSimplified = douglasPeucker(begin, std::next(max_dist_it), epsilon);
  auto secondLineSimplified = douglasPeucker(max_dist_it, end, epsilon);

  // Combine the two lines into one line and return the merged line.
  // Note that we have to skip the first point of the second line, as it is duplicated above.
  firstLineSimplified.insert(firstLineSimplified.end(),
    std::make_move_iterator(std::next(secondLineSimplified.begin())),
    std::make_move_iterator(secondLineSimplified.end()));
  return firstLineSimplified;
}

} // end namespace

namespace costmap_converter
{

CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH() : BaseCostmapToPolygons()
{
  costmap_ = NULL;
  // dynamic_recfg_ = NULL;
  neighbor_size_x_ = neighbor_size_y_ = -1;
  offset_x_ = offset_y_ = 0.;
}

CostmapToPolygonsDBSMCCH::~CostmapToPolygonsDBSMCCH()
{
//  if (dynamic_recfg_ != NULL)
//    delete dynamic_recfg_;
}

void CostmapToPolygonsDBSMCCH::initialize(rclcpp::Node::SharedPtr nh)
{
    BaseCostmapToPolygons::initialize(nh);

    costmap_ = NULL;

    parameter_.max_distance_ = 0.1;
    //nh->get_parameter_or<double>("cluster_max_distance", parameter_.max_distance_, parameter_.max_distance_);

    parameter_.min_pts_ = 2;
    nh->get_parameter_or<int>("cluster_min_pts", parameter_.min_pts_, parameter_.min_pts_);

    parameter_.max_pts_ = 30;
    nh->get_parameter_or<int>("cluster_max_pts", parameter_.max_pts_, parameter_.max_pts_);

    parameter_.min_keypoint_separation_ = 0.1;
    nh->get_parameter_or<double>("convex_hull_min_pt_separation", parameter_.min_keypoint_separation_, parameter_.min_keypoint_separation_);

    std::string static_map_polygons_string;
    nh->declare_parameter("static_map_lines", rclcpp::ParameterValue("NULL"));
    nh->get_parameter("static_map_lines", static_map_polygons_string);

    if (static_map_polygons_string.compare("NULL") != 0) {
      std::string error_return;
      parameter_buffered_ = parameter_;
      std::vector<std::vector<float>> array;
      array = nav2_costmap_2d::parseVVF(static_map_polygons_string, error_return);
      setPolygon(array);
    } else {
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

void CostmapToPolygonsDBSMCCH::compute()
{
  /*
    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::msg::Polygon>());

    geometry_msgs::msg::Point32 p;
    geometry_msgs::msg::Polygon poly;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    poly.points.push_back(p);
    p.x = 3.0;
    poly.points.push_back(p);
    p.y = 2.0;
    poly.points.push_back(p);
    p.x = 0.0;
    poly.points.push_back(p);
    polygons->push_back(poly);
    poly.points.clear();

    p.x = 0.9;
    p.y = 0.0;
    poly.points.push_back(p);
    p.y = 0.18;
    poly.points.push_back(p);
    polygons->push_back(poly);
    poly.points.clear();

    p.x = 1.53;
    p.y = 0.0;
    poly.points.push_back(p);
    p.y = 0.3;
    poly.points.push_back(p);
    polygons->push_back(poly);
    poly.points.clear();

    p.x = 2.1;
    p.y = 0.0;
    poly.points.push_back(p);
    p.y = 0.18;
    poly.points.push_back(p);
    polygons->push_back(poly);
    poly.points.clear();

    // replace shared polygon container
    updatePolygonContainer(polygons);*/
}

void CostmapToPolygonsDBSMCCH::setCostmap2D(nav2_costmap_2d::Costmap2D *costmap)
{
    if (!costmap)
      return;

    costmap_ = costmap;

    updateCostmap2D();
}

void CostmapToPolygonsDBSMCCH::updateCostmap2D()
{
      occupied_cells_.clear();

      if (!costmap_->getMutex())
      {
        RCLCPP_ERROR(getLogger(), "Cannot update costmap since the mutex pointer is null");
        return;
      }

      // TODO: currently dynamic reconigure is not supported in ros2
      { // get a copy of our parameters from dynamic reconfigure
        std::lock_guard<std::mutex> lock(parameter_mutex_);
        parameter_ = parameter_buffered_;
      }

      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());

      // allocate neighbor lookup
      int cells_x = int(costmap_->getSizeInMetersX() / parameter_.max_distance_) + 1;
      int cells_y = int(costmap_->getSizeInMetersY() / parameter_.max_distance_) + 1;

      if (cells_x != neighbor_size_x_ || cells_y != neighbor_size_y_) {
        neighbor_size_x_ = cells_x;
        neighbor_size_y_ = cells_y;
        neighbor_lookup_.resize(neighbor_size_x_ * neighbor_size_y_);
      }
      offset_x_ = costmap_->getOriginX();
      offset_y_ = costmap_->getOriginY();
      for (auto& n : neighbor_lookup_)
        n.clear();

      // get indices of obstacle cells
      for(std::size_t i = 0; i < costmap_->getSizeInCellsX(); i++)
      {
        for(std::size_t j = 0; j < costmap_->getSizeInCellsY(); j++)
        {
          int value = costmap_->getCost(i,j);
          if(value >= nav2_costmap_2d::LETHAL_OBSTACLE)
          {
            double x, y;
            costmap_->mapToWorld((unsigned int)i, (unsigned int)j, x, y);
            addPoint(x, y);
          }
        }
      }
}


void CostmapToPolygonsDBSMCCH::dbScan(std::vector< std::vector<KeyPoint> >& clusters)
{
  std::vector<bool> visited(occupied_cells_.size(), false);

  clusters.clear();

  //DB Scan Algorithm
  int cluster_id = 0; // current cluster_id
  clusters.push_back(std::vector<KeyPoint>());
  for(int i = 0; i< (int)occupied_cells_.size(); i++)
  {
    if(!visited[i]) //keypoint has not been visited before
    {
      visited[i] = true; // mark as visited
      std::vector<int> neighbors;
      regionQuery(i, neighbors); //Find neighbors around the keypoint
      if((int)neighbors.size() < parameter_.min_pts_) //If not enough neighbors are found, mark as noise
      {
        clusters[0].push_back(occupied_cells_[i]);
      }
      else
      {
        ++cluster_id; // increment current cluster_id
        clusters.push_back(std::vector<KeyPoint>());

        // Expand the cluster
        clusters[cluster_id].push_back(occupied_cells_[i]);
        for(int j = 0; j<(int)neighbors.size(); j++)
        {
          if ((int)clusters[cluster_id].size() == parameter_.max_pts_)
            break;

          if(!visited[neighbors[j]]) //keypoint has not been visited before
          {
            visited[neighbors[j]] = true;  // mark as visited
            std::vector<int> further_neighbors;
            regionQuery(neighbors[j], further_neighbors); //Find more neighbors around the new keypoint
//             if(further_neighbors.size() < min_pts_)
//             {
//               clusters[0].push_back(occupied_cells[neighbors[j]]);
//             }
//             else
            if ((int)further_neighbors.size() >= parameter_.min_pts_)
            {
              // neighbors found
              neighbors.insert(neighbors.end(), further_neighbors.begin(), further_neighbors.end());  //Add these newfound P' neighbour to P neighbour vector "nb_indeces"
              clusters[cluster_id].push_back(occupied_cells_[neighbors[j]]);
            }
          }
        }
      }
    }
  }
}

void CostmapToPolygonsDBSMCCH::regionQuery(int curr_index, std::vector<int>& neighbors)
{
    RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "region query");

    neighbors.clear();

    double dist_sqr_threshold = parameter_.max_distance_ * parameter_.max_distance_;
    const KeyPoint& kp = occupied_cells_[curr_index];
    int cx, cy;
    pointToNeighborCells(kp, cx,cy);

    // loop over the neighboring cells for looking up the points
    const int offsets[9][2] = {{-1, -1}, {0, -1}, {1, -1},
                               {-1,  0}, {0,  0}, {1,  0},
                               {-1,  1}, {0,  1}, {1,  1}};
    for (int i = 0; i < 9; ++i)
    {
      int idx = neighborCellsToIndex(cx + offsets[i][0], cy + offsets[i][1]);
      if (idx < 0 || idx >= int(neighbor_lookup_.size()))
        continue;
      const std::vector<int>& pointIndicesToCheck = neighbor_lookup_[idx];
      for (int point_idx : pointIndicesToCheck) {
        if (point_idx == curr_index) // point is not a neighbor to itself
          continue;
        const KeyPoint& other = occupied_cells_[point_idx];
        double dx = other.x - kp.x;
        double dy = other.y - kp.y;
        double dist_sqr = dx*dx + dy*dy;
        if (dist_sqr <= dist_sqr_threshold)
          neighbors.push_back(point_idx);
      }
    }
}

bool isXCoordinateSmaller(const CostmapToPolygonsDBSMCCH::KeyPoint& p1, const CostmapToPolygonsDBSMCCH::KeyPoint& p2)
{
  return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
}

void CostmapToPolygonsDBSMCCH::convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon)
{
    //Monotone Chain ConvexHull Algorithm source from http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull
    RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "convex hull");

    int k = 0;
    int n = cluster.size();

    // sort points according to x coordinate (TODO. is it already sorted due to the map representation?)
    std::sort(cluster.begin(), cluster.end(), isXCoordinateSmaller);

    polygon.points.resize(2*n);

    // lower hull
    for (int i = 0; i < n; ++i)
    {
      while (k >= 2 && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0)
      {
        --k;
      }
      cluster[i].toPointMsg(polygon.points[k]);
      ++k;
    }

    // upper hull
    for (int i = n-2, t = k+1; i >= 0; --i)
    {
      while (k >= t && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0)
      {
        --k;
      }
      cluster[i].toPointMsg(polygon.points[k]);
      ++k;
    }


    polygon.points.resize(k); // original
    // TEST we skip the last point, since in our definition the polygon vertices do not contain the start/end vertex twice.
//     polygon.points.resize(k-1); // TODO remove last point from the algorithm above to reduce computational cost

    simplifyPolygon(polygon);
}



void CostmapToPolygonsDBSMCCH::convexHull2(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon)
{
    RCLCPP_WARN(rclcpp::get_logger("costmap_converter_custom"), "convex hull 2");

    std::vector<KeyPoint>& P = cluster;
    std::vector<geometry_msgs::msg::Point32>& points = polygon.points;

    // Sort P by x and y
    std::sort(P.begin(), P.end(), isXCoordinateSmaller);

    // the output array H[] will be used as the stack
    int i;                 // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    double xmin = P[0].x;
    for (i = 1; i < (int)P.size(); i++)
        if (P[i].x != xmin) break;
    minmax = i - 1;
    if (minmax == (int)P.size() - 1)
    {   // degenerate case: all x-coords == xmin
        points.push_back(geometry_msgs::msg::Point32());
        P[minmin].toPointMsg(points.back());
        if (P[minmax].y != P[minmin].y) // a  nontrivial segment
        {
            points.push_back(geometry_msgs::msg::Point32());
            P[minmax].toPointMsg(points.back());
        }
        // add polygon endpoint
        points.push_back(geometry_msgs::msg::Point32());
        P[minmin].toPointMsg(points.back());
        return;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = (int)P.size() - 1;
    double xmax = P.back().x;
    for (i = P.size() - 2; i >= 0; i--)
        if (P[i].x != xmax) break;
    maxmin = i+1;

    // Compute the lower hull on the stack H
    // push  minmin point onto stack
    points.push_back(geometry_msgs::msg::Point32());
    P[minmin].toPointMsg(points.back());
    i = minmax;
    while (++i <= maxmin)
    {
        // the lower line joins P[minmin]  with P[maxmin]
        if (cross(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            continue;           // ignore P[i] above or on the lower line

        while (points.size() > 1)         // there are at least 2 points on the stack
        {
            // test if  P[i] is left of the line at the stack top
            if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            points.pop_back();         // pop top point off  stack
        }
        // push P[i] onto stack
        points.push_back(geometry_msgs::msg::Point32());
        P[i].toPointMsg(points.back());
    }

    // Next, compute the upper hull on the stack H above  the bottom hull
    if (maxmax != maxmin)      // if  distinct xmax points
    {
         // push maxmax point onto stack
         points.push_back(geometry_msgs::msg::Point32());
         P[maxmax].toPointMsg(points.back());
    }
    int bot = (int)points.size();                  // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
        // the upper line joins P[maxmax]  with P[minmax]
        if (cross( P[maxmax], P[minmax], P[i])  >= 0 && i > minmax)
            continue;           // ignore P[i] below or on the upper line

        while ((int)points.size() > bot)     // at least 2 points on the upper stack
        {
            // test if  P[i] is left of the line at the stack top
            if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            points.pop_back();         // pop top point off stack
        }
        // push P[i] onto stack
        points.push_back(geometry_msgs::msg::Point32());
        P[i].toPointMsg(points.back());
    }
    if (minmax != minmin)
    {
        // push  joining endpoint onto stack
        points.push_back(geometry_msgs::msg::Point32());
        P[minmin].toPointMsg(points.back());
    }

    simplifyPolygon(polygon);
}

void CostmapToPolygonsDBSMCCH::simplifyPolygon(geometry_msgs::msg::Polygon& polygon)
{
  /*
  size_t triangleThreshold = 3;
  // check if first and last point are the same. If yes, a triangle has 4 points
  if (polygon.points.size() > 1
      && std::abs(polygon.points.front().x - polygon.points.back().x) < 1e-5
      && std::abs(polygon.points.front().y - polygon.points.back().y) < 1e-5)
  {
    triangleThreshold = 4;
  }
  if (polygon.points.size() <= triangleThreshold) // nothing to do for triangles or lines
    return;
  // TODO Reason about better start conditions for splitting lines, e.g., by
  // https://en.wikipedia.org/wiki/Rotating_calipers
  polygon.points = douglasPeucker(polygon.points.begin(), polygon.points.end(), parameter_.min_keypoint_separation_);;
  */
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

//void CostmapToPolygonsDBSMCCH::reconfigureCB(CostmapToPolygonsDBSMCCHConfig& config, uint32_t level)
//{
  //boost::mutex::scoped_lock lock(parameter_mutex_);
  //parameter_buffered_.max_distance_ = config.cluster_max_distance;
  //parameter_buffered_.min_pts_ = config.cluster_min_pts;
  //parameter_buffered_.max_pts_ = config.cluster_max_pts;
  //parameter_buffered_.min_keypoint_separation_ = config.convex_hull_min_pt_separation;
//}

}//end namespace costmap_converter