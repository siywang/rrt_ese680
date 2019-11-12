// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>


// Struct defining the Node object in the RRT tree.
// More fields could be added to this struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Publisher map_pub;
    ros::Publisher drive_pub;
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;

    //map stuff
    nav_msgs::OccupancyGrid gridMap_static;
    nav_msgs::OccupancyGrid gridMap_dynamic;
    nav_msgs::OccupancyGrid gridMap_final;
    int lookahead_dist = 0; // lookahead distance
    int inf = 0; // inflation size

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params
    geometry_msgs::Pose car_pose_msg; // car's current location


    // random generator, use this
    // https://stackoverflow.com/questions/39288595/why-not-just-use-random-device
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<double> x_dist;
    std::uniform_real_distribution<double> y_dist;

    //map methods
    int row_col_to_index(int row, int col);
    geometry_msgs::PointStamped transform(double x, double y);

    // callbacks
    // where rrt actually happens
    //void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void pf_callback(const nav_msgs::Odometry ::ConstPtr &odom_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

