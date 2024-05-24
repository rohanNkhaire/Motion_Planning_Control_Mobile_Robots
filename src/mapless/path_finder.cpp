#include "diff_drive_package/mapless/path_finder.hpp"

PathFinder::PathFinder() : nh_(""), pnh_("~")
{
	// Publisher
	pub_twist_cmd_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	// Subscriber
	sub_pose_ = nh_.subscribe("odom", 1, &PathFinder::callbackPose, this);
	sub_laser_ = nh_.subscribe("scan", 1, &PathFinder::callbackLaser, this);

		// Timer
	timer_control_ = nh_.createTimer(ros::Duration(dt), &PathFinder::timerCallback, this);

	if (debug)
	{
		pub_goal_vec_ = nh_.advertise<geometry_msgs::PoseStamped>("debug/goal_vector", 1);
		pub_obstacle_vec_ =  nh_.advertise<geometry_msgs::PoseStamped>("debug/obstacle_vector", 1);
		pub_resultant_vec_ =  nh_.advertise<geometry_msgs::PoseStamped>("debug/resultant_vector", 1);
		pub_cw_vec_ =  nh_.advertise<geometry_msgs::PoseStamped>("debug/cw_vector", 1);
		pub_ccw_vec_ =  nh_.advertise<geometry_msgs::PoseStamped>("debug/ccw_vector", 1);
	}

	// Get the Goal Point from rosparams
	float goal_x, goal_y;
	if(nh_.getParam("goal_x", goal_x) && nh_.getParam("goal_y", goal_y))
	{
		goal_pt.x = goal_x;
		goal_pt.y = goal_y;
		goal_set = true;
	}

}

void PathFinder::callbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Get the laser scan
	robot_laser_ = msg;
	scan_recieved = true;
}

void PathFinder::callbackPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Get the robot pose
	robot_odom_ = msg;
	odom_recieved = true;
}

void PathFinder::timerCallback(const ros::TimerEvent &te)
{
	if (goal_set && odom_recieved && scan_recieved)
	{
		// Get the goal vector
		Node goal_vec = getGoalVector(robot_odom_, goal_pt);

		// Get a resultant obstacle vector
		Node obstacle_vec = getObstacleVector(robot_laser_, robot_odom_);

		// Get Follow surface vector
		std::pair<Node, Node> wall_vectors = getWallVector(obstacle_vec);

		// Blend goal and obstacle vectors
		Node goal_with_avoidance = combineGoalObstacle(goal_vec, obstacle_vec);

		// Get angle between two vectors
		double vec_angle = getAngleBetweenVectors(goal_vec, obstacle_vec);
		double CW_vec_angle = getAngleBetweenVectors(goal_vec, wall_vectors.first);
		double CCW_vec_angle = getAngleBetweenVectors(goal_vec, wall_vectors.second);

		// This is the final vec we follow
		Node resultant_dir;

		// Get minimum distance between obstacle and robot
		double min_obstacle_distance = calculateMinDistance(robot_laser_);

		// GEt euclidean distance from goal to robot pose
		double goal_dist = euclideanDistance(robot_odom_->pose.pose.position.x, robot_odom_->pose.pose.position.y,
																					goal_pt.x, goal_pt.y);

		// Switch between following wall and combined goal + obstacle
		// Initially we folow goal
		if (init_state_ || (vec_angle <= M_PI_2 || goal_dist < 0.75) && min_obstacle_distance > distance_threshold)
		{
			resultant_dir = goal_vec;
			init_state_ = false;

		}
		// This is a follow wall
		// if angle between obstacel and goal becomes greater than 90 and dist thershold is not vialated
		// we follow wall
		else if (vec_angle > M_PI_2 && min_obstacle_distance > distance_threshold)
		{
			double CW_vec_angle = getAngleBetweenVectors(goal_vec, wall_vectors.first);
			double CCW_vec_angle = getAngleBetweenVectors(goal_vec, wall_vectors.second);
			if(CW_vec_angle <= CCW_vec_angle)
			{
				resultant_dir = wall_vectors.first;
			}
			else
			{
				resultant_dir = wall_vectors.second;
			}
		}
		// We do blended obstacle avoidance and goal behaviour
		else
		{
			resultant_dir = goal_with_avoidance;
		}

		// Generate Control Commands
		std::pair<float, float> twist_cmd = generateTwist(robot_odom_, resultant_dir);

		// Publish Command
		publishCmd(twist_cmd);

		// debug pose for all the vectors
		// goal vec
		// blended goal and obstacle avoidance
		// CCW and Cw to the obstacle vector
		if (debug)
		{
			publishVectorViz(goal_vec, pub_goal_vec_);
			publishVectorViz(resultant_dir, pub_resultant_vec_);
			publishVectorViz(obstacle_vec, pub_obstacle_vec_);
			publishVectorViz(wall_vectors.first, pub_cw_vec_);
			publishVectorViz(wall_vectors.second, pub_ccw_vec_);
		}
	}

}

// Get the goal vector betwween robot pose nad goal
PathFinder::Node PathFinder::getGoalVector(const nav_msgs::Odometry::ConstPtr& odom, const Node& goal)
{
	Node goal_direction;
	float e_x = goal.x - odom->pose.pose.position.x;
	float e_y = goal.y - odom->pose.pose.position.y;
	float e_th = atan2(e_y, e_x);

	goal_direction.x = e_x;
	goal_direction.y = e_y;
	goal_direction.th = e_th;

	return goal_direction;
}

// Get the obstacle vector using all the range of the scanner
PathFinder::Node PathFinder::getObstacleVector(const sensor_msgs::LaserScan::ConstPtr& laser_scan, 
																								const nav_msgs::Odometry::ConstPtr& odom)
{
	Node resultant_obstacle_vector;
	resultant_obstacle_vector.x = 0.0;
	resultant_obstacle_vector.y = 0.0;

	// Iterate through the scanner angle and generate a resultant vector of all the osbtacle points
	for (size_t i = 0; i < laser_scan->ranges.size(); ++i)
	{
		if (laser_scan->ranges[i] < laser_scan->range_max && laser_scan->ranges[i] > laser_scan->range_min)
		{
			Node range_node, range_node_odom;
			float angle = laser_scan->angle_min + i*laser_scan->angle_increment;
			range_node.x = laser_scan->ranges[i] * cos(angle);
			range_node.y = laser_scan->ranges[i] * sin(angle);

			// Convert points to odom frame
			geometry_msgs::PointStamped in_msg, out_msg;
			in_msg.header = laser_scan->header;
			in_msg.point.x = range_node.x;
			in_msg.point.y = range_node.y;

  			try
  			{
					tf_listener_.waitForTransform("odom", laser_scan->header.frame_id,
        	laser_scan->header.stamp, ros::Duration(0.05));		
  			  tf_listener_.transformPoint("odom", in_msg, out_msg);
  			}
  			catch (tf::TransformException ex)
  			{
  			  ROS_ERROR("%s", ex.what());
  			}

			// sum all the range x and y
			//resultant_obstacle_vector.x += (odom->pose.pose.position.x - out_msg.point.x)*((1/laser_scan->ranges[i])*0.5);
			//resultant_obstacle_vector.y += (odom->pose.pose.position.y - out_msg.point.y)*((1/laser_scan->ranges[i])*0.5);
			// use gain to scale the obstacles that are closer
			double obs_gain = 1/laser_scan->ranges[i]*(0.2/pow(laser_scan->ranges[i], 2) + 0.03);
			resultant_obstacle_vector.x += (odom->pose.pose.position.x - out_msg.point.x)*obs_gain;
			resultant_obstacle_vector.y += (odom->pose.pose.position.y - out_msg.point.y)*obs_gain;
			}
	}

	resultant_obstacle_vector.th = atan2(resultant_obstacle_vector.y, resultant_obstacle_vector.x);

	return resultant_obstacle_vector;
}

// Blend the goal and the obstacle vector
PathFinder::Node PathFinder::combineGoalObstacle(const Node& goal, const Node& obstacle)
{
	Node resultant_dir;
	resultant_dir.x = (1 - alpha)*obstacle.x + alpha*goal.x;
	resultant_dir.y = (1 - alpha)*obstacle.y + alpha*goal.y;

	resultant_dir.th = atan2(resultant_dir.y, resultant_dir.x);

	return resultant_dir;
}

// Get the +90 and -90 vectors from the obstacel vectors
// Use rotation matrix on the obstacle vector
std::pair<PathFinder::Node, PathFinder::Node> PathFinder::getWallVector(const Node& obstacle_vec)
{
	Node CW_wall_vec, CCW_wall_vec;
	CW_wall_vec.x = obstacle_vec.y;
	CW_wall_vec.y = -obstacle_vec.x;
	CW_wall_vec.th = atan2(CW_wall_vec.y, CW_wall_vec.x);

	CCW_wall_vec.x = -obstacle_vec.y;
	CCW_wall_vec.y = obstacle_vec.x;
	CCW_wall_vec.th = atan2(CCW_wall_vec.y, CCW_wall_vec.x);

	return std::make_pair(CW_wall_vec, CCW_wall_vec);
}

// Generate the twist and yaw commands
// use PID controller for Yaw tracking
// Use a gain function for Vel taht is dependent on the distance of the robot to the goal pt
std::pair<float, float> PathFinder::generateTwist(const nav_msgs::Odometry::ConstPtr& odom, const Node& res_vec)
{
	float omega, vel;

	// Get Yaw from quaternion of the robot
	float theta = tf::getYaw(odom->pose.pose.orientation);
	float e_th = res_vec.th - theta;
	e_th = atan2(sin(e_th), cos(e_th));

	// Use PID to control yaw
  float e_I = prev_e_I + e_th*dt;
  float e_D = (e_th - prev_e_th)/dt;

	omega = K_p*e_th + K_d*e_D + K_i*e_I;

	// Get vel from the vector
	// restrict vel to 0.26 m/s
	double goal_dist = euclideanDistance(odom->pose.pose.position.x, odom->pose.pose.position.y,
																					goal_pt.x, goal_pt.y);
	double v_gain = (0.26*(1 - exp(-v_coeff*pow(goal_dist, 2))))/goal_dist;
	vel = v_gain*goal_dist;

	prev_e_I = e_I;
	prev_e_th = e_th;

	return std::make_pair(vel, omega);
}

// Publish the twist command
void PathFinder::publishCmd(const std::pair<float, float>& twistcmd)
{
	geometry_msgs::Twist control_cmd;

	control_cmd.linear.x = twistcmd.first;
	control_cmd.angular.z = twistcmd.second;

	pub_twist_cmd_.publish(control_cmd);
}

// Get the euclidean distance
double PathFinder::euclideanDistance(const Node& n1, const Node& n2)
{
	return sqrt(pow((n2.x - n1.x), 2) + pow((n2.y - n1.y), 2));
}

double PathFinder::euclideanDistance(const double& x1, const double& y1, const double& x2, const double& y2)
{
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

// Publish teh debug msgs
void PathFinder::publishVectorViz(const Node& req_node, ros::Publisher& ros_pub)
{
	geometry_msgs::PoseStamped req_pose;
	req_pose.header.frame_id = "odom";
	req_pose.header.stamp = ros::Time::now();
	req_pose.pose.position.x = robot_odom_->pose.pose.position.x;
	req_pose.pose.position.y = robot_odom_->pose.pose.position.y;
	req_pose.pose.orientation = tf::createQuaternionMsgFromYaw(req_node.th);

	ros_pub.publish(req_pose);

}

// get angle between vecotrs using dot product
double PathFinder::getAngleBetweenVectors(const Node& n1, const Node& n2)
{
	double mul = n1.x * n2.x + n1.y * n2.y;
	double n1_norm = sqrt(n1.x*n1.x + n1.y*n1.y);
	double n2_norm = sqrt(n2.x*n2.x + n2.y*n2.y);

	return acos(mul/(n1_norm*n2_norm));
}

// Calculate the min range of the laser scanner
double PathFinder::calculateMinDistance(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
	double min_distance = INT_MAX;
	for (size_t i = 0; i < laser_scan->ranges.size(); ++i)
	{
		if (laser_scan->ranges[i] < laser_scan->range_max && laser_scan->ranges[i] > laser_scan->range_min)
		{
			double distance = laser_scan->ranges[i];
			if (distance < min_distance)
			{
				min_distance = distance;
			}
		}
	}

	return min_distance;
}	