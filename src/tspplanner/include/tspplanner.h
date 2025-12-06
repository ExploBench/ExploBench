#ifndef TSPPLANNER_H
#define TSPPLANNER_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef ORTOOLS_FOUND
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>
#endif

using MapSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr;
using OdomSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;
using PCDPublisherPtr =
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
using PathPublisherPtr = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Timer = rclcpp::TimerBase::SharedPtr;
using Pose = geometry_msgs::msg::Pose;
using Path = nav_msgs::msg::Path;

struct Point2D {
  int x, y;
  bool operator<(const Point2D &other) const {
    if (x != other.x)
      return x < other.x;
    return y < other.y;
  }
  bool operator==(const Point2D &other) const {
    return x == other.x && y == other.y;
  }
};

struct Node2D {
  int x, y;
  double g, h, f;
  std::shared_ptr<Node2D> parent;
  Node2D(int x_, int y_, double g_ = 0.0)
      : x(x_), y(y_), g(g_), h(0.0), f(g + h), parent(nullptr){};
  int64_t to_key(int width) const {
    return static_cast<int64_t>(y) * width + x;
  };
};

struct NodeComparator {
  bool operator()(const Node2D &a, const Node2D &b) const { return a.f > b.f; }
};

class TSPPlanner : public rclcpp::Node {
public:
  TSPPlanner();

private:
  // subscribers and publishers
  MapSubscriberPtr visible_sub;
  OdomSubscriberPtr odom_sub;
  PCDPublisherPtr frontier_pub;
  PathPublisherPtr path_pub;
  Timer planning_timer;

  // map data
  OccupancyGrid visible, visible_cost;

  // map parameters
  float resolution, origin_x, origin_y;
  int width, height;
  bool visible_received;
  std::mutex map_mutex;

  // robot state
  Pose current_pose;
  Point2D robot_grid_pos{0, 0};

  // configuration
  double replan_interval = 2.0; // seconds
  int cluster_size = 6;

  // frontier data
  std::vector<Point2D> frontiers;
  std::vector<Point2D> raw_frontiers;
  std::vector<std::pair<int, int>> neighbor_dirs;

  // TSP related
  std::vector<std::vector<double>> cost_matrix;
  std::vector<Point2D> tsp_ordered_frontiers;

  // callbacks
  void visibleCallback(const OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void planningCallback();

  // map processing
  OccupancyGrid generateCostmap(const OccupancyGrid &original_map);
  
  // frontier detection
  std::vector<Point2D> findFrontiers();
  bool isFrontier(const std::vector<signed char> &map_data, int x, int y);
  
  // frontier clustering
  std::vector<Point2D>
  clusterFrontiers(const std::vector<Point2D> &raw_frontiers, int cluster_size);
  void moveFrontiersAwayFromObstacle();

  // path planning
  bool aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                   std::vector<Point2D> &path);
  double computePathCost(const std::vector<Point2D> &path);

  // TSP solving
  void buildCostMatrix();
  bool solveTSP();
  std::vector<Point2D> buildPathFromTSP();

  // utility functions
  void worldToGrid(const Pose &pose, int &grid_x, int &grid_y);
  void gridToWorld(int grid_x, int grid_y, double &world_x, double &world_y);
  void publishFrontierCloud();
  void publishPath(const std::vector<Point2D> &path);
};

#endif


