#include "diff_drive_package/map/astar.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "astar_search");
  Astar astar_finder;
  ros::spin();
  return 0;
}