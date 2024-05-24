#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <queue>
#include <memory>

struct Node
{
	std::array<int, 2> index;
  bool visited = false;  // node status
  double gc = 0.0;         // actual cost
  double hc = 0.0;         // heuristic cost
  Node* parent = nullptr; // parent node

  double cost() const { return gc + hc; }
};

struct compareOP
{
bool operator()(const std::shared_ptr<Node> n1, const std::shared_ptr<Node> n2)
	{
		return n1->cost() > n2->cost();
	}
};

class Astar
{
	public:
		Astar();

	private:
  	ros::NodeHandle nh_;                    // ros node handle
  	ros::NodeHandle pnh_;                   // Private ros node handle

		// Publishers
  	ros::Publisher pub_twist_cmd_;          // Publisher twist command
		ros::Publisher pub_resultant_vec_;
		ros::Publisher pub_viz_msg_;
		// Subscribers
		ros::Subscriber sub_costmap_;							// Subscribe to 2D laser scanner
		ros::Subscriber sub_odom_;								// Subscribe robot odom

		// Timer
		ros::Timer timer_control_;              // Timer for control command computation

		// Callbacks
		void publishTraj(const nav_msgs::OccupancyGrid::ConstPtr &);
		void callbackCostmap(const grid_map_msgs::GridMap::ConstPtr &);
		void callbackPose(const nav_msgs::Odometry::ConstPtr &);
		void timerCallback(const ros::TimerEvent &);

		// Variables
		bool debug = true;
		double dt = 0.01;
		geometry_msgs::Pose final_goal_pose_;
		bool goal_set = false;
		nav_msgs::Odometry robot_pose_;
		grid_map::GridMap occupancy_map_;
		tf::TransformListener tf_listener_;
		int map_size_x, map_size_y;
		std::vector<std::vector<int>> neighbors = {{0,1}, {0,-1}, {1,0}, {-1,0}, {1,1}, {1,-1}, {-1, 1},{-1,-1}};
		double v_coeff = 25.0;
		bool costmap_ready_ = false;
		bool odom_recieved_ = false;

		// PID vars
		float K_p = 1.25;
		float K_d = 0.5;
		float K_i = 0.0;
		double prev_e_I = 0.0;
		double prev_e_th = 0.0;

		// Functions
		std::vector<std::shared_ptr<Node>> search(const nav_msgs::Odometry&, const geometry_msgs::Pose&);
		geometry_msgs::Pose intermediateGoalPose(const grid_map::GridMap&);
		nav_msgs::Path generateWaypoints(std::vector<Node*>, const grid_map::GridMap&);
		std::pair<float, float> generateTwist(const nav_msgs::Path&);
		void publishTwistCmd(const std::pair<float, float>&);
		void publishVectorViz(const double&, ros::Publisher&);
		double euclideanDistance(const double&, const double&, const double&, const double&);
		bool checkValid(std::array<int, 2>);
		double heuristicCost(const std::shared_ptr<Node>&, std::array<int, 2>);
		std::vector<Node*> getShortestPath(std::vector<std::shared_ptr<Node>>&);
		bool checkSame(const std::shared_ptr<Node>&, std::array<int, 2>);
		void createLocalPathMarker(const nav_msgs::Path&, visualization_msgs::MarkerArray&);

};    