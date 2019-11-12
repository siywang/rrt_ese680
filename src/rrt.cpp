// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh) : nh_(nh), gen((std::random_device()) ()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic;

    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("inflation", inf);
    nh_.getParam("lookahead", lookahead_dist);

//    nh_.getParam("frame_id", frame_id);
//    nh_.getParam("height",height);
//    nh_.getParam("width",width);
//    nh_.getParam("resolution",resolution);


    // ROS publishers // TODO: create publishers for the the drive topic, and other topics you might need
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 10);
    map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("gridMap", 1, true);

    // ROS subscribers // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    // TODO: create a occupancy grid
    boost::shared_ptr<nav_msgs::MapMetaData const> levine_metadata_ptr;
    nav_msgs::MapMetaData levine_metadata_msg;
    levine_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (levine_metadata_ptr != nullptr) { levine_metadata_msg = *levine_metadata_ptr; }

    boost::shared_ptr<nav_msgs::OccupancyGrid const> levine_map_ptr;
    levine_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");

    gridMap_static.data = levine_map_ptr->data;
    gridMap_static.header.frame_id = "map";
    gridMap_static.header.stamp = ros::Time::now();
    gridMap_static.info.height = levine_metadata_msg.height;
    gridMap_static.info.width = levine_metadata_msg.width;
    gridMap_static.info.resolution = levine_metadata_msg.resolution;
    gridMap_static.info.origin = levine_metadata_msg.origin;
    //gridMap.data.assign(gridMap.info.height * gridMap.info.width, 0);

    gridMap_dynamic.header.frame_id = "map";
    gridMap_dynamic.header.stamp = ros::Time::now();
    gridMap_dynamic.info.height = levine_metadata_msg.height;
    gridMap_dynamic.info.width = levine_metadata_msg.width;
    gridMap_dynamic.info.resolution = levine_metadata_msg.resolution;
    gridMap_dynamic.info.origin = levine_metadata_msg.origin;
    gridMap_dynamic.data.assign(gridMap_dynamic.info.height * gridMap_dynamic.info.width, 0);

    gridMap_final.header.frame_id = "map";
    gridMap_final.header.stamp = ros::Time::now();
    gridMap_final.info.height = levine_metadata_msg.height;
    gridMap_final.info.width = levine_metadata_msg.width;
    gridMap_final.info.resolution = levine_metadata_msg.resolution;
    gridMap_final.info.origin = levine_metadata_msg.origin;
    gridMap_final.data.assign(gridMap_final.info.height * gridMap_final.info.width, 0);

    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    /*
     * The scan callback, update your occupancy grid here
     * Args:
     * scan_msg (*LaserScan): pointer to the incoming scan message
     * TODO: update your occupancy grid
     */

    std::vector<float> ranges = scan_msg->ranges;
    // the following is for dynamic layer
    for (int i = 180; i < 901; i++) { // 180 degrees in front of the car
        double range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue; // remove nan and inf
        float angles = scan_msg->angle_min + scan_msg->angle_increment * (float) i;
        double x = range * cos((double) angles), y = range * sin((double) angles); // in laser frame

        // now transform   transform(x,y)
        int col = static_cast<int>((transform(x, y).point.x - gridMap_dynamic.info.origin.position.x) /gridMap_dynamic.info.resolution); // cell
        int row = static_cast<int>((transform(x, y).point.y - gridMap_dynamic.info.origin.position.y) /gridMap_dynamic.info.resolution); // cell

        //inflation
        for (int j = -inf; j <= inf; j++) {
            for (int k = -inf; k <= inf; k++) {
                int col_inf = col - k;
                int row_inf = row - j; // inf: inflation
                int index_inf = row_col_to_index(row_inf, col_inf);// 1D index
               //gridMap_dynamic.data[index_inf] = range > lookahead_dist ? 0 : 100;
               gridMap_dynamic.data[index_inf] = 100;
            }
        }
    }

    // find out the union set between dynamic and static layer
    for(int k = 1; k <= gridMap_dynamic.info.width * gridMap_dynamic.info.height; k++ ){
        gridMap_final.data[k] = (gridMap_static.data[k] || gridMap_dynamic.data[k] != 0) ? 100:0; // mark obstacles
    }

    map_pub.publish(gridMap_final); // publish the final grid map

    gridMap_dynamic.data.assign(gridMap_dynamic.info.height * gridMap_dynamic.info.width, 0); // clear obstacles after passing it
    std::cout << "height: " << gridMap_dynamic.info.height << std::endl;
    std::cout << "width: " << gridMap_dynamic.info.width << std::endl;
    std::cout << "resolution: " << gridMap_dynamic.info.resolution << std::endl;
    std::cout << "origin: " << gridMap_dynamic.info.origin << std::endl;

}


geometry_msgs::PointStamped RRT::transform(double x, double y) {
    /* transformation between /laser and /map frames
     * Input: x anf y of laser scan in laser frame
     * Return: transformed point in map frame
     */
    geometry_msgs::PointStamped before_transform;
    before_transform.header.frame_id = "/laser";
    before_transform.point.x = x;
    before_transform.point.y = y;

    geometry_msgs::PointStamped after_transform;
    after_transform.header.frame_id = "/map";
    try {
        listener.transformPoint("/map", before_transform, after_transform);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return after_transform; // return transformed point
}

int RRT::row_col_to_index(int row, int col) {
    return row * gridMap_dynamic.info.width + col; // 1D index in grid map
}


void RRT::pf_callback(const nav_msgs::Odometry ::ConstPtr &odom_msg) { //geometry_msgs::PoseStamped::ConstPtr &pose_msg
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    car_pose_msg = odom_msg->pose.pose;



    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop



    // path found as Path message

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space
    // TODO: fill in this method
    // car's current position: ??
    double currentX = car_pose_msg.position.x; //vehicle pose X
    double currentY = car_pose_msg.position.y; //vehicle pose Y

    std::vector<double> sampled_point;


    int x = x_dist(gen);
    int y = y_dist(gen);

    sampled_point.push_back(x);
    sampled_point.push_back(y);

    //

   std::cout << "???????" << x << std::endl;




    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<Node> found_path;
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}
