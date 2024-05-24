#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <memory>

class PathFinder
{
public:
  PathFinder();

private:
  ros::NodeHandle nh_;                    // ros node handle
  ros::NodeHandle pnh_;                   // Private ros node handle

	// Publishers
  ros::Publisher pub_twist_cmd_;          // Publisher twist command
	ros::Publisher pub_goal_vec_;
	ros::Publisher pub_obstacle_vec_;
	ros::Publisher pub_resultant_vec_;
	ros::Publisher pub_cw_vec_;
	ros::Publisher pub_ccw_vec_;

	// Subscribers
  ros::Subscriber sub_pose_;              // Subscribe to current pose
	ros::Subscriber sub_laser_;							// Subscribe to 2D laser scanner

	// Timer
	ros::Timer timer_control_;              // Timer for control command computation

	// Callbacks
	void publishTwist(const double &, const double &);
	void callbackPose(const nav_msgs::Odometry::ConstPtr &);
	void callbackLaser(const sensor_msgs::LaserScan::ConstPtr &);
	void timerCallback(const ros::TimerEvent &);

	struct Node
	{
		float x;
		float y;
		float th;
	};

	// Variables
	double dt = 0.02;
	Node goal_pt;
	bool goal_set = false;
	bool debug = true;
	tf::TransformListener tf_listener_;
	bool scan_recieved = false;
	bool odom_recieved = false;
	double distance_threshold = 0.6;

	// States
	bool init_state_ = true;
	bool follow_goal_ = false;
	bool follow_wall_ = false;
	bool avoid_obstacle_ = false;

	// Params
	const float alpha = 0.4;

	// PID
	float K_p = 1.25;
	float K_d = 0.5;
	float K_i = 0.0;
	float prev_e_th = 0.0;
	float prev_e_I = 0.0;
	double v_coeff = 25.0;
	double rad_buffer = M_1_PI;


	nav_msgs::Odometry::ConstPtr robot_odom_;
	sensor_msgs::LaserScan::ConstPtr robot_laser_;

	// Functions
	Node getGoalVector(const nav_msgs::Odometry::ConstPtr&, const Node&);
	Node getObstacleVector(const sensor_msgs::LaserScan::ConstPtr&, 
													const nav_msgs::Odometry::ConstPtr&);
	std::pair<Node, Node> getWallVector(const Node& obstacle_vec);												
	Node combineGoalObstacle(const Node&, const Node&);
	std::pair<float, float> generateTwist(const nav_msgs::Odometry::ConstPtr&, const Node&);
	void publishCmd(const std::pair<float, float>&);
	void publishVectorViz(const Node&, ros::Publisher&);
	double euclideanDistance(const Node&, const Node&);
	double euclideanDistance(const double&, const double&, const double&, const double&);
	double getAngleBetweenVectors(const Node&, const Node&);
	double calculateMinDistance(const sensor_msgs::LaserScan::ConstPtr&);
};