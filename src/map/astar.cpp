#include "diff_drive_package/map/astar.hpp"

Astar::Astar() : nh_(""), pnh_("~")
{
  // Publisher
	pub_twist_cmd_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pub_viz_msg_ = nh_.advertise<visualization_msgs::MarkerArray>("local_path", 1);

	// Subscriber
	sub_costmap_ = nh_.subscribe("grid_occupancy_map", 1, &Astar::callbackCostmap, this);
	sub_odom_ = nh_.subscribe("odom", 1, &Astar::callbackPose, this);

	// Timer
	timer_control_ = nh_.createTimer(ros::Duration(dt), &Astar::timerCallback, this);

	// Get the Goal Point from rosparams
	float goal_x, goal_y;
	if(nh_.getParam("goal_x", goal_x) && nh_.getParam("goal_y", goal_y))
	{
		final_goal_pose_.position.x = goal_x;
		final_goal_pose_.position.y = goal_y;
		final_goal_pose_.position.z = 0.0;
		goal_set = true;
	}

	// debug topics
	if (debug)
	{
		pub_resultant_vec_ =  nh_.advertise<geometry_msgs::PoseStamped>("debug/resultant_vector", 1);
	}
}

void Astar::callbackCostmap(const grid_map_msgs::GridMap::ConstPtr& msg)
{
	// Get the gird map and convert it to ros type
	grid_map::GridMapRosConverter::fromMessage(*msg, occupancy_map_);

	// set the No. of indices in the map
	map_size_x = occupancy_map_.getSize().x();
	map_size_y = occupancy_map_.getSize().y();

	costmap_ready_ = true;
}

void Astar::callbackPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	// Get robot pose using odom
	robot_pose_ = *msg;

	odom_recieved_ = true;
}

void Astar::timerCallback(const ros::TimerEvent &te)
{
	// Only execute if odom, map, and goal are set
	if (goal_set && costmap_ready_ && odom_recieved_)
	{
		// Get intermediate goal
		// Since it is a local costmap, we need intermediate goals to reach the final destination
		geometry_msgs::Pose inter_goal = intermediateGoalPose(occupancy_map_);
		ROS_INFO("Goal Pose: %f, %f", inter_goal.position.x, inter_goal.position.y);

		// Generate a path from start to goal
		// Use Astar search algorithm to generate a path from start to intermediate goal
		std::vector<std::shared_ptr<Node>> local_path = search(robot_pose_, inter_goal);

		// Get shortest path
		// Iterate throught the parents of the nodes to get the shortest path
		std::vector<Node*> shortest_path = getShortestPath(local_path);
		ROS_INFO("Path size is: %li" ,shortest_path.size());

		// Generate waypoints
		nav_msgs::Path odom_path = generateWaypoints(shortest_path, occupancy_map_);

		// Generate control commands for waypoint following
		std::pair<float, float> twist_cmd = generateTwist(odom_path);

		// Publish Twist CMD to turtlebot
		publishTwistCmd(twist_cmd);


		// Visualize path and goal
		visualization_msgs::MarkerArray path_marker_array;
		createLocalPathMarker(odom_path, path_marker_array);

		pub_viz_msg_.publish(path_marker_array);

	}

}

geometry_msgs::Pose Astar::intermediateGoalPose(const grid_map::GridMap& grid_map)
{
	geometry_msgs::Pose inter_goal_pose;
	geometry_msgs::PointStamped in_msg, out_msg;
	in_msg.header = robot_pose_.header;
	in_msg.point = final_goal_pose_.position;
	// Check if final goal is in the occupancy grid
	try
  {
		tf_listener_.waitForTransform("base_scan", "odom",
  	robot_pose_.header.stamp, ros::Duration(0.05));		
    tf_listener_.transformPoint("base_scan", in_msg, out_msg);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

	grid_map::Index goal_idx;
	grid_map::Position goal_pt(out_msg.point.x, out_msg.point.y);

	double distance = euclideanDistance(final_goal_pose_.position.x, final_goal_pose_.position.y,
																				robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y);

	// check if goal index is in the grid map and dist between goal and origin is less than 1.0m
	if (grid_map.getIndex(goal_pt, goal_idx) && distance < 1.0)
	{
		inter_goal_pose.position = out_msg.point;
		return inter_goal_pose;
	}

	// Generate intermediate goal along the goal direction
	// Iterate thorught the grid map to obtain point that has the lowest cost
	// Cost is calculated with euclidean distance and cost of the node in the grid
	double total_cost = INT_MAX;
	grid_map::Matrix& data = occupancy_map_["laser_layer"];
  for (grid_map::GridMapIterator iterator(occupancy_map_); !iterator.isPastEnd(); ++iterator)
	{
		// search for a pt with low cost to goal and low grid cost
		const int i = iterator.getLinearIndex();
		if (data(i) >= 0.0 && data(i) < 1e6)
		{
			grid_map::Position curr_pt;
			occupancy_map_.getPosition(*iterator, curr_pt);

			// Get cost
			double euclidean_cost = euclideanDistance(out_msg.point.x, out_msg.point.y, curr_pt(0), curr_pt(1));
			double grid_cost = data(i);
			double cost = euclidean_cost + grid_cost;
			if (cost < total_cost)
			{
				total_cost = cost;
				inter_goal_pose.position.x = curr_pt(0);
				inter_goal_pose.position.y = curr_pt(1); 
			}
		}
	}
	

	return inter_goal_pose;
}

std::vector<std::shared_ptr<Node>> Astar::search(const nav_msgs::Odometry& odom, const geometry_msgs::Pose& goal_pose)
{
	// Store a 2D vector for distances and visisted nodes
	std::vector<std::vector<double>> dist(map_size_x, std::vector<double>(map_size_y, 100000.0));
	std::vector<std::vector<double>> visited(map_size_x, std::vector<double>(map_size_y, false));
  std::vector<std::shared_ptr<Node>> path;

	// Priority que to sort for lowest cost
	std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, compareOP> priorityQ;
	
	// Get start index in occupancy grid
	grid_map::Index start_idx;
	grid_map::Position start_pos(0.0, 0.0);
	occupancy_map_.getIndex(start_pos, start_idx);
	std::array<int, 2> start_arr_idx = {start_idx(0), start_idx(1)};

	// Get goal index in occupancy grid
	grid_map::Index goal_idx;
	grid_map::Position goal_pos(goal_pose.position.x, goal_pose.position.y);
	occupancy_map_.getIndex(goal_pos, goal_idx);
	std::array<int, 2> goal_arr_idx = {goal_idx(0), goal_idx(1)};

	// Start the search
	std::shared_ptr<Node> first = std::make_shared<Node>();
  first->index = start_arr_idx;
  first->gc = 0;
  first->hc = heuristicCost(first, goal_arr_idx);
  first->parent = nullptr;
	priorityQ.push(first);
	dist[first->index[0]][first->index[1]] = 0.0;
	
	// Search unitl que is empty
	while (!priorityQ.empty())
	{
    auto curr_node = priorityQ.top();
		priorityQ.pop();
    path.push_back(curr_node);
    visited[curr_node->index[0]][curr_node->index[1]] = true;
		// exit if we found the goal
		if (checkSame(curr_node, goal_arr_idx))
    { 
        return path;
    } 
		// Search throguh neighbors along 8 directions
		// up down left right and 4 diagonals
		for (auto& idx : neighbors)
		{
			std::array<int, 2> neighbor = {curr_node->index[0] + idx[0], curr_node->index[1] + idx[1]};
			if (!visited[neighbor[0]][neighbor[1]] && checkValid(neighbor))
			{
				double curr_dist = curr_node->gc + 1.0;
				if (curr_dist < dist[neighbor[0]][neighbor[1]])
				{
					dist[neighbor[0]][neighbor[1]] = curr_dist;
					std::shared_ptr<Node> adj_node = std::make_shared<Node>();
          adj_node->index = neighbor;
          adj_node->gc = curr_dist;
					adj_node->hc = heuristicCost(adj_node, goal_arr_idx);
          adj_node->parent = curr_node.get();
					priorityQ.push(adj_node);
				}
			}
		}
	}

	return path;
}

// Get the shortest path by iterating through the parents of each node
std::vector<Node*> Astar::getShortestPath(std::vector<std::shared_ptr<Node>>& path)
{
		std::vector<Node*> short_path;
    auto curr_node = path.back().get();
    while (curr_node != nullptr)
    {
      short_path.emplace_back(curr_node);
      curr_node = curr_node->parent;
    }
		// Reverse the list to get the waypoints for start to goal
		std::reverse(short_path.begin(), short_path.end());

		return short_path;
}

// Add euclidean heuristic cost
// Add cost in the grid
double Astar::heuristicCost(const std::shared_ptr<Node>& node, std::array<int, 2> dst)
{
	double heuristic_cost;

	// Add euclidean heuristics
	double hdistance_cost = sqrt(pow(node->index[0] - dst[0], 2) + pow(node->index[1] - dst[1], 2));

	// Add grid cost of the cell
	grid_map::Index node_idx(node->index[0], node->index[1]);
	double hgrid_cost = occupancy_map_.at("laser_layer", node_idx);
	// csafety check for costs that are nan or inf
	if (std::isnan(hgrid_cost)) hgrid_cost = 0.0;

	if (std::isinf(hgrid_cost)) hgrid_cost = 1e6;

	ROS_INFO("Grid cost is: %f", hgrid_cost);
	heuristic_cost = hdistance_cost + hgrid_cost;

  return heuristic_cost;
}

// generate waypoints from the shortest path
nav_msgs::Path Astar::generateWaypoints(std::vector<Node*> path, const grid_map::GridMap& costmap)
{
	nav_msgs::Path waypoints;
	// Iterate through the path 
	// Store waypoins as nav_msgs::Path
	for (size_t i = 0; i < path.size(); ++i)
	{
		geometry_msgs::PoseStamped wayp;
		grid_map::Position curr_pt;
		grid_map::Index curr_idx(path[i]->index.at(0), path[i]->index.at(1));
		costmap.getPosition(curr_idx, curr_pt);

		// Transform waypoints to odom frame
		geometry_msgs::PointStamped in_msg, out_msg;
		in_msg.header.frame_id = "base_scan";
		in_msg.header.stamp = robot_pose_.header.stamp;
		in_msg.point.x = curr_pt(0);
		in_msg.point.y = curr_pt(1);
		try
  	{
			tf_listener_.waitForTransform("odom", "base_scan",
  		robot_pose_.header.stamp, ros::Duration(0.05));		
  	  tf_listener_.transformPoint("odom", in_msg, out_msg);
  	}
  	catch (tf::TransformException ex)
  	{
  	  ROS_ERROR("%s", ex.what());
  	}

		wayp.pose.position = out_msg.point;
		waypoints.poses.emplace_back(wayp);

		}

	waypoints.header.frame_id = "odom";
	waypoints.header.stamp = ros::Time::now();

	return waypoints;
}

// generate the twist command and yaw
// Track the Yaw using PID controller
// Use an exp function for V to increase or reduce speed based on euclidean distance
// between goal and robot pose
std::pair<float, float> Astar::generateTwist(const nav_msgs::Path& waypoints)
{
	float omega, vel;

	// Get closest waypoint to the robot pose
	double max_dist = INT_MAX;
	int req_idx;
	for (int i = 0; i < waypoints.poses.size(); ++i)
	{
		double distance = euclideanDistance(robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y,
																				waypoints.poses[i].pose.position.x, waypoints.poses[i].pose.position.y);
		
		if (distance < max_dist)
		{
			max_dist = distance;
			req_idx = i;
		}
	}

	// Get Yaw from quaternion of the robot
	float theta = tf::getYaw(robot_pose_.pose.pose.orientation);

	// get yaw from waypoint
	int wayp_size = waypoints.poses.size() - 1;
	int forward_idx = req_idx + 5;
	int potential_idx = std::min(wayp_size, forward_idx);
	double dx = waypoints.poses[potential_idx].pose.position.x - waypoints.poses[req_idx].pose.position.x;
	double dy = waypoints.poses[potential_idx].pose.position.y - waypoints.poses[req_idx].pose.position.y;
	float wayp_th = atan2(dy, dx);

	// Get angle for error tracking
	float e_th = wayp_th - theta;
	e_th = atan2(sin(e_th), cos(e_th));

	if (debug) publishVectorViz(e_th, pub_resultant_vec_);

	// Use PID to control yaw
  float e_I = prev_e_I + e_th*dt;
  float e_D = (e_th - prev_e_th)/dt;

	omega = K_p*e_th + K_d*e_D + K_i*e_I;

	// Get vel from the vector
	// restrict vel to 0.26 m/s
	double goal_dist = euclideanDistance(robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y,
																					final_goal_pose_.position.x, final_goal_pose_.position.y);
	double v_gain = (0.26*(1 - exp(-v_coeff*pow(goal_dist, 2))))/goal_dist;
	vel = v_gain*goal_dist;

	prev_e_I = e_I;
	prev_e_th = e_th;

	if (goal_dist < 0.05)
	{
		vel = 0.0;
		omega = 0.0;
	}

	return std::make_pair(vel, omega);
}

// Publish the Twist command
void Astar::publishTwistCmd(const std::pair<float, float>& twistcmd)
{
	geometry_msgs::Twist control_cmd;

	control_cmd.linear.x = twistcmd.first;
	control_cmd.angular.z = twistcmd.second;

	pub_twist_cmd_.publish(control_cmd);
}

// Check if the index is in the grid map
bool Astar::checkValid(std::array<int, 2> node)
{
	if (node[0] < 0 || node[0] > map_size_x || node[1] < 0 || node[1] > map_size_y) return false;

	return true;
}

// Debug publish Pose msgs
void Astar::publishVectorViz(const double& req_yaw, ros::Publisher& ros_pub)
{
	geometry_msgs::PoseStamped req_pose;
	req_pose.header.frame_id = "odom";
	req_pose.header.stamp = ros::Time::now();
	req_pose.pose.position.x = robot_pose_.pose.pose.position.x;
	req_pose.pose.position.y = robot_pose_.pose.pose.position.y;
	req_pose.pose.orientation = tf::createQuaternionMsgFromYaw(req_yaw);

	ros_pub.publish(req_pose);

}

// Get euclidean distance
double Astar::euclideanDistance(const double& x1, const double& y1, const double& x2, const double& y2)
{
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}
// check if the indices are the same
bool Astar::checkSame(const std::shared_ptr<Node>& n1, std::array<int, 2> n2)
{
	if (n1->index[0] == n2[0] && n1->index[1] == n2[1]) return true;

	return false;
}

// A visualizer of the local path
void Astar::createLocalPathMarker(const nav_msgs::Path& waypoints_array, visualization_msgs::MarkerArray& markerArray)
{
  std_msgs::ColorRGBA total_color;
  total_color.r = 1.0;
  total_color.g = 0.2;
  total_color.b = 0.3;
  total_color.a = 1.0;
   
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "odom";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.ns = "local_trajectory_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.02;
  lane_waypoint_marker.scale.y = 0.02;
  lane_waypoint_marker.color = total_color;
  lane_waypoint_marker.frame_locked = true;

  for (unsigned int i=0; i<waypoints_array.poses.size() - 1; i++)
  {

    geometry_msgs::Point point;
    point = waypoints_array.poses.at(i).pose.position;
    lane_waypoint_marker.points.push_back(point);

  }

	markerArray.markers.push_back(lane_waypoint_marker);

	// Goal Point
	visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "odom";
  goal_marker.header.stamp = ros::Time::now();
  goal_marker.ns = "goal_marker";
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.scale.x = 0.1;
  goal_marker.scale.y = 0.1;
	goal_marker.scale.z = 0.1;
  goal_marker.color = total_color;
  goal_marker.frame_locked = true;
	goal_marker.pose = waypoints_array.poses.back().pose;

	markerArray.markers.push_back(goal_marker);

}