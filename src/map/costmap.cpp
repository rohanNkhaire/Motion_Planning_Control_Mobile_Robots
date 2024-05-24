#include "diff_drive_package/map/costmap.hpp"


// Geth euclidean distance between two points
double euclideanDistance(const double& x1, const double& y1, const double& x2, const double& y2)
{
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

CostmapGenerator::CostmapGenerator() : nh_(""), pnh_("~")
{
  // Publisher
	pub_occupancy_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("nav_occupancy_map", 1);
  pub_grid_map_ = nh_.advertise<grid_map_msgs::GridMap>("grid_occupancy_map", 1);

	// Subscriber
	sub_laser_ = nh_.subscribe("scan", 1, &CostmapGenerator::callbackLaser, this);

	// Initialize the costmap
	initializeCostmap();
	world_frame_id_ = std::string("odom");
  laser_frame_id_ = std::string("base_scan");
}

void CostmapGenerator::callbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Update the occupancy map with laser scanner findings
	updateCostmap(map_, *msg);

  // Set same header for both maps
  std_msgs::Header header_msg;
  header_msg.frame_id = laser_frame_id_;
  header_msg.stamp = ros::Time::now();

  // Publish Occupancy and Grid maps
  publishCostmap(map_, header_msg);
  publishGridmap(map_, header_msg);
}

void CostmapGenerator::initializeCostmap()
{
  map_.setFrameId(laser_frame_id_);
  map_.setGeometry(grid_map::Length(6.0, 6.0), 0.04,
                       grid_map::Position(0.0, 0.0));
	map_.add("laser_layer");
}

void CostmapGenerator::updateCostmap(const grid_map::GridMap& grid_costmap, const sensor_msgs::LaserScan& scan)
{
  // Update occupancy.
  // Init map to -1 values at the start
  map_.add("laser_layer");

  // Go through each angle of the laser to find the obstaclesin the map
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment);
    // Use Bresenham Algorithm to he the grid intersection with the laser line
    std::vector<geometry_msgs::Point> points_intensity_vec = getObstacles(grid_costmap, scan, angle, scan.ranges[i]);
    // Use the points to fill the Occupancy grid map
    map_["laser_layer"] = fillOccupancyMap(points_intensity_vec, grid_costmap, scan.ranges[i]);
  }

}

void CostmapGenerator::publishCostmap(const grid_map::GridMap& grid_map, const std_msgs::Header& header)
{
  // Publish the cost map
  nav_msgs::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, "laser_layer", 0.0, 100.0,
                                                 out_occupancy_grid);
  out_occupancy_grid.header = header;
  pub_occupancy_map_.publish(out_occupancy_grid);
}

void CostmapGenerator::publishGridmap(const grid_map::GridMap& grid_map, const std_msgs::Header& header)
{
  // Publish the grid map
  grid_map_msgs::GridMap out_gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(grid_map, out_gridmap_msg);
  out_gridmap_msg.info.header = header;
  pub_grid_map_.publish(out_gridmap_msg);
}
	
std::vector<geometry_msgs::Point> CostmapGenerator::getObstacles(const grid_map::GridMap& grid_map, const sensor_msgs::LaserScan& scan, const double& angle, const double& range)
{
    // Get the start position and end position of the laser in the base_scan frame
    double range_val = std::min(range, grid_map.getLength().x()/2.0);
    double map_border_x = range_val*cos(angle);
    double map_border_y = range_val*sin(angle);
    double x = grid_map.getPosition().x() + map_border_x;
    double y = grid_map.getPosition().y() + map_border_y;

    // Apply Bresenham algorithm in all 4 quadrants to get the gird cells
    std::vector<geometry_msgs::Point> line_pts = bresenhamLineAlgo(grid_map.getPosition().x(), grid_map.getPosition().y(), x, y);                                                                     

    return line_pts;
}

grid_map::Matrix CostmapGenerator::fillOccupancyMap(std::vector<geometry_msgs::Point>& pts, const grid_map::GridMap& cost_map, const double& range)
{
  double intensity = 0.0;
  geometry_msgs::Point ref_pt = pts.back();
  grid_map::Matrix gridmap_data = cost_map["laser_layer"];

  // For all the points outputted by Bresenham algorithm
  // assign intensity such that near obstacle points get high value
  // farther from obstacle point gets low value
  for (auto& pt : pts)
  {
    grid_map::Position curr_pt(pt.x, pt.y);
    grid_map::Index curr_idx;
    bool idx_exist = cost_map.getIndex(curr_pt, curr_idx);
    double dist = euclideanDistance(pt.x, pt.y, ref_pt.x, ref_pt.y);
    intensity = (1/dist)*(10.0/(pow(dist, 2) + 0.01));
    
    //if (range > 3.5) intensity = 0.0;
    if (gridmap_data(curr_idx(0), curr_idx(1)) < intensity || std::isnan(gridmap_data(curr_idx(0), curr_idx(1))))
    {
      gridmap_data(curr_idx(0), curr_idx(1)) = intensity;
    }
  }

  // Experimental function at the time of submission
  // To add intensity values perpendicular to the laser angle for corners
  addPerpendicularCost(gridmap_data, ref_pt);

  return gridmap_data;
}

// Get the perpendicular angle from the laser angle
// Apply Bresenham to a point of arbitrary distance 
// Fill in the intensity
void CostmapGenerator::addPerpendicularCost(grid_map::Matrix& data, const geometry_msgs::Point& ref_pt)
{
  std::pair<std::array<double, 3>, std::array<double, 3>> perp_vec = getPerpendicularVector(ref_pt);
  std::vector<geometry_msgs::Point> pepr_pts_vec;
  // Get arbitrary dist pts
  double dist_proj = 0.6;
  double x_perp_1 = ref_pt.x + dist_proj*cos(perp_vec.first.at(2));
  double y_perp_1 = ref_pt.y + dist_proj*cos(perp_vec.first.at(2));
  double x_perp_2 = ref_pt.x + dist_proj*cos(perp_vec.second.at(2));
  double y_perp_2 = ref_pt.y + dist_proj*cos(perp_vec.second.at(2));

  grid_map::Position curr_pt_1(x_perp_1, y_perp_1);
  grid_map::Index curr_idx_1;
  bool idx_exist_1 = map_.getIndex(curr_pt_1, curr_idx_1);

  grid_map::Position curr_pt_2(x_perp_2, y_perp_2);
  grid_map::Index curr_idx_2;
  bool idx_exist_2 = map_.getIndex(curr_pt_2, curr_idx_2);

  // check if idx in grid map for both the perpendicular vectors
  if (idx_exist_1)
  {
    if (!std::isnan(data(curr_idx_1(0), curr_idx_1(1))))
    {
      pepr_pts_vec = bresenhamLineAlgo(ref_pt.x, ref_pt.y, x_perp_1, y_perp_1);
      ROS_INFO("CW");
    }
  }

  if (idx_exist_2)
  {
    if (!std::isnan(data(curr_idx_2(0), curr_idx_2(1))))
    {
      pepr_pts_vec = bresenhamLineAlgo(ref_pt.x, ref_pt.y, x_perp_2, y_perp_2);
      ROS_INFO("CCW");
    }
  }
  // Fill in the intensity for these indices/points
  for (auto& pt : pepr_pts_vec)
  {
    grid_map::Position curr_pt(pt.x, pt.y);
    grid_map::Index curr_idx;
    bool idx_exist = map_.getIndex(curr_pt, curr_idx);
    if (idx_exist)
    {
      double dist = euclideanDistance(pt.x, pt.y, ref_pt.x, ref_pt.y);
      double intensity = (1/dist)*(5.0/(pow(dist, 2) + 0.1));
      data(curr_idx(0), curr_idx(1)) = intensity;
    }
    
  }
  
}

// Bresenham algorithm for 4 quadrants
std::vector<geometry_msgs::Point> CostmapGenerator::bresenhamLineAlgo(const double& x1, const double& y1, const double& x2, const double& y2)
{
  std::vector<geometry_msgs::Point> output_vec;
  geometry_msgs::Point init_pt;
  bool reverse_vec = false;
  double x, y, xe, ye;

  // Calcualte the delta
  double dx = x2 - x1;
  double dy = y2 - y1;

  // Get the abs
  double dx_a = abs(dx);
  double dy_a = abs(dy);

  // Error interval for both axis
  double px = 2*dy_a - dx_a;
  double py = 2*dx_a - dy_a;
  
  // Line is in X-axis
  if (dx_a >= dy_a)
  {
    // Line is drawn left to right
    if (dx >= 0)
    {
      x = x1;
      y = y1;
      xe = x2;
    }
    else // Line drawn right t oleft
    {
      reverse_vec = true;
      x = x2;
      y = y2;
      xe = x1;
    }
    
    // add the first point to hte output vector
    init_pt.x = x;
    init_pt.y = y;
    output_vec.emplace_back(init_pt);

    // Loop to increment x and y
    for (size_t i = 0; x < xe; ++i)
    {
      geometry_msgs::Point pt;
      x = x + 0.01;
      // Lookout for octants in the quadrant in X-axis
      if (px < 0)
      {
        px = px + 2*dy_a;
      }
      else
      {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
        {
          y = y + 0.01;
        }
        else
        {
          y = y - 0.01;
        }
        px = px + 2* (dy_a - dx_a);
      }
      pt.x = x; pt.y = y;
      output_vec.emplace_back(pt);
    }
  }
  else
  {
    // Line drawn bottom to top
    if (dy >= 0)
    {
      x = x1; 
      y = y1; 
      ye = y2;
    } 
    else 
    { 
      reverse_vec = true;
      x = x2; 
      y = y2; 
      ye = y1;
    }
    init_pt.x = x; 
    init_pt.y = y;
    output_vec.emplace_back(init_pt);
    // Loop to incremetn x and y
    for (size_t i = 0; y < ye; ++i) 
    {
      geometry_msgs::Point pt;
      y = y + 0.01;     
      // Lookout for octants in the Y-axis       
      if (py <= 0) {
        py = py + 2 * dx_a;
      } 
      else 
      {
        if ((dx < 0 && dy<0) || (dx > 0 && dy > 0)) {
            x = x + 0.01;
        } else {
            x = x - 0.01;
        }
        py = py + 2 * (dx_a - dy_a);
      }            
      pt.x = x;
      pt.y = y;
      output_vec.emplace_back(pt);
    }
  }

  // we reverse so as to maintain the start point from the origin/ gird centre
  if (reverse_vec)
  {
    std::reverse(output_vec.begin(), output_vec.end());
  }

  return output_vec;
}

// get the perpendicualr vector by rotating the centre-to-obstacle vector
std::pair<std::array<double, 3>, std::array<double, 3>> CostmapGenerator::getPerpendicularVector(const geometry_msgs::Point& obstacle_vec)
{
  // apply -90 and 90 deg rotation using rotation matrix
	std::array<double, 3> CW_wall_vec, CCW_wall_vec;
	CW_wall_vec[0] = obstacle_vec.y;
	CW_wall_vec[1] = -obstacle_vec.x;
	CW_wall_vec[2] = atan2(CW_wall_vec[1], CW_wall_vec[0]);

	CCW_wall_vec[0] = -obstacle_vec.y;
	CCW_wall_vec[1] = obstacle_vec.x;
	CCW_wall_vec[2] = atan2(CCW_wall_vec[1], CCW_wall_vec[0]);

	return std::make_pair(CW_wall_vec, CCW_wall_vec);
}