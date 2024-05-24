#include "diff_drive_package/map/costmap.hpp"

int main(int argc, char** argv)
{
  // Initialize the classs and run
  ros::init(argc, argv, "costmap_generator");
  CostmapGenerator costmap;
  ros::spin();
  return 0;
}