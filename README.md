# Motion Planning and COntrol of Mobile Robot #

### Map navigation ###
![](media/map_navigation.mp4)

### Mapless navigation ###
![](media/mapless_navigation.mp4)

## Objective ##
This repo applies Motion Planning and Control algorithms to move the mobile robot to a goal destination.

## Dependencies ##
- [grid_map](https://github.com/ANYbotics/grid_map) is used for generating and manipulating the occupancy map.


## Installation ##
```bash

# Create a worksspace of your own and make a src folder
mkdir -p test_ws/src
cd test_ws/src

# Clone the repo
git clone https://github.com/rohanNkhaire/Motion_Planning_Control_Mobile_Robots.git

# Build the workspace
cd ../../

# Install dependency
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
```

## Run an experiment ##

```bash
# Launch the Gazebo env
roslaunch diff_drive_package launch_diff_drive_robot.launch

# Run the mapless navigation OR
# Run the navigation with map

# Mapless navigation
rosrun diff_drive_package mapless_navigation

# Map navigation
rosrun diff_drive_package costmap_generator
rosrun diff_drive_package astar_navigation
```

## Note ##

The setup consists of a mobile robot with a 2D laser scanner.

There are two algorithms
- With Map
    - Creates local occupancy grid map.
    - Applies Astar to find a path.

- Without Map
    - Uses Vectors for goal and obstacles to move.    