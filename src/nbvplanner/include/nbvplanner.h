#ifndef NBVPLANNER_H
#define NBVPLANNER_H

#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

using OdomSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;
using MapSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr;
using PathPublisherPtr = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;
using MarkerPublisherPtr =
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;

struct RRTNode {
  Eigen::Vector3d position;
  double gain;
  RRTNode *parent;
  RRTNode(const Eigen::Vector3d &pos, double g, RRTNode *par = nullptr)
      : position(pos), gain(g), parent(par) {}
};

class NBVPlanner : public rclcpp::Node {
private:
  struct ConfigSpace {
    double x_min, x_max, y_min, y_max; // unit: meter
    ConfigSpace(double xmin, double xmax, double ymin, double ymax)
        : x_min(xmin), x_max(xmax), y_min(ymin), y_max(ymax) {}
  };

  ConfigSpace traversable_space;

  // robot name
  std::string robot_name;

  // map related
  bool map_received;
  nav_msgs::msg::OccupancyGrid map, costmap;
  double origin_x, origin_y;
  double height, width;
  double resolution;
  MapSubscriberPtr map_sub;
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void generateCostmap(const nav_msgs::msg::OccupancyGrid &map,
                       nav_msgs::msg::OccupancyGrid &costmap);

  // odom related
  bool odom_ready;
  Eigen::Vector3d robot_position;
  OdomSubscriberPtr odom_sub;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // path related
  nav_msgs::msg::Path rrt_path;
  PathPublisherPtr path_pub;
  void publishPath(nav_msgs::msg::Path path);

  // planning related
  double replan_distance;
  double step_size;
  int max_iterations;
  size_t max_nodes;
  double segment_travel_limit;
  Eigen::Vector3d goal;
  Eigen::Vector3d execution_goal;
  Eigen::Vector3d last_plan_position;
  double current_segment_length;
  bool has_active_segment;
  void planningCallback();
  // void planning();

  // rrt
  bool force_rebuild = false;
  size_t N_tol;
  std::vector<RRTNode *> rrt_tree;
  // random number generator
  std::mt19937 rng;
  RRTNode *root;
  RRTNode *best_node;
  std::vector<Eigen::Vector3d> retained_branch;
  bool exploration_solved;
  void buildTree();
  void clearTree();
  void seedTreeFromRetainedBranch();
  Eigen::Vector3d sampleRandomPoint();
  RRTNode *nearestNode(const Eigen::Vector3d &sample);
  double computeGain(const Eigen::Vector3d &point);
  Eigen::Vector3d steer(const Eigen::Vector3d &from, const Eigen::Vector3d &to);
  bool isPointFree(const Eigen::Vector3d &point);
  bool isPathFree(const Eigen::Vector3d &from, const Eigen::Vector3d &to);

  bool constructPath(Eigen::Vector3d goal, nav_msgs::msg::Path &path);
  std::vector<Eigen::Vector3d> extractBranch(RRTNode *node);
  size_t selectExecutionPrefix(const std::vector<Eigen::Vector3d> &branch,
                               std::vector<Eigen::Vector3d> &prefix,
                               double &prefix_length);
  nav_msgs::msg::Path clipPathByDistance(const nav_msgs::msg::Path &path,
                                         double max_distance,
                                         double &clipped_length);

  // tree visualization
  MarkerPublisherPtr tree_pub, best_node_pub;
  void publishBestNodeVisualization();
  void publishTreeVisualization();

  // timing
  rclcpp::TimerBase::SharedPtr planning_timer;

public:
  NBVPlanner();
  ~NBVPlanner();
};

#endif