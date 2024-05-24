#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>

class CostmapGenerator
{
	public:
		CostmapGenerator();

	private:
  ros::NodeHandle nh_;                    // ros node handle
  ros::NodeHandle pnh_;                   // Private ros node handle

	// Publishers
  ros::Publisher pub_occupancy_map_;          // Publish Occupancy map
  ros::Publisher pub_grid_map_;               // Publish Grid map

	// Subscribers
	ros::Subscriber sub_laser_;							// Subscribe to 2D laser scanner

	// Callbacks
	void callbackLaser(const sensor_msgs::LaserScan::ConstPtr &);

	// Variables
	tf::TransformListener tf_listener_;
  std::string world_frame_id_; //!< frame_id of the world frame
  std::string laser_frame_id_;  //!< map frame id in tf
  bool has_frame_id_;  //!< true if map frame_id was already set

	// map params
	grid_map::GridMap map_; //!< local map with fixed orientation
	int width = 200;
	int height = 200;
	int resolution = 0.5;
	
	// Functions
	void initializeCostmap();
	void updateCostmap(const grid_map::GridMap&, const sensor_msgs::LaserScan&);
  void publishCostmap(const grid_map::GridMap&, const std_msgs::Header&);
  void publishGridmap(const grid_map::GridMap&, const std_msgs::Header&);
	std::vector<geometry_msgs::Point> bresenhamLineAlgo(const double&, const double&, const double&, const double&);
	grid_map::Matrix fillOccupancyMap(std::vector<geometry_msgs::Point>&, const grid_map::GridMap&, const double&);
	std::vector<geometry_msgs::Point> getObstacles(const grid_map::GridMap&, const sensor_msgs::LaserScan&, const double&, const double&);
	void addPerpendicularCost(grid_map::Matrix&, const geometry_msgs::Point&);
	std::pair<std::array<double, 3>, std::array<double, 3>> getPerpendicularVector(const geometry_msgs::Point&);

};
