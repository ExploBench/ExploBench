#ifndef FIND_NEAREST_H
#define FIND_NEAREST_H

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

class FindNearest : public rclcpp::Node {
public:
  FindNearest();

private:
  // subscribers and publishers
  MapSubscriberPtr visible_sub;
  OdomSubscriberPtr odom_sub;
  PCDPublisherPtr frontier_pub;
  PCDPublisherPtr raw_frontier_pub;
  PathPublisherPtr path_pub;
  Timer timer;

  // map data
  OccupancyGrid visible, visible_cost;

  // map parameters
  float resolution, origin_x, origin_y;
  int width, height;
  bool visible_received;
  std::mutex map_mutex;

  // robot state
  Pose current_pose;
  Point2D target_frontier{0, 0};
  Point2D start_point{0, 0};
  bool replan_flag = true;

  // configuration
  double distance_threshold = 3.0;
  double travel_distance_threshold = 2.0;

  // frontier data
  std::vector<Point2D> frontiers;
  std::vector<Point2D> raw_frontiers;
  std::set<Point2D> visited_frontiers;
  std::vector<std::pair<int, int>> neighbor_dirs;

  // callbacks
  void visibleCallback(const OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  OccupancyGrid generateCostmap(const OccupancyGrid &original_map);
  std::vector<Point2D> findFrontiers();
  std::vector<Point2D>
  clusterFrontiers(const std::vector<Point2D> &raw_frontiers, int cluster_size);
  void moveFrontiersAwayFromObstacle();

  // path planning
  bool aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                   std::vector<Point2D> &path);
  void assignNearestFrontier();

  // utility functions
  bool isFrontier(const std::vector<signed char> &map_data, int x, int y);
  bool checkArriveTarget();
  bool checkTravelFarEnough();
  void worldToGrid(const Pose &pose, int &grid_x, int &grid_y);
  void publishFrontierCloud();
  void publishRawFrontierCloud();
  void publishPath(const std::vector<Point2D> &path);
};

#endif