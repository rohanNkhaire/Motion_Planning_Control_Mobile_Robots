#include "diff_drive_package/mapless/path_finder.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "mapless_navigation");
  PathFinder mapless_pathfinder;
  ros::spin();
  return 0;
}