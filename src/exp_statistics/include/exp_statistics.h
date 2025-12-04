#ifndef EXP_STATISTICS_H
#define EXP_STATISTICS_H

#include <cmath>
#include <fstream>
#include <iomanip>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#define DEBUG_INFO false

using OccupancyGridSubPtr =
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr;
using MarkerSubPtr =
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr;

class ExpStatistics : public rclcpp::Node {
public:
  ExpStatistics();
  ~ExpStatistics();

private:
  std::ofstream output_file;
  std::string file_path;

  bool map_received;
  nav_msgs::msg::OccupancyGrid current_map;
  double start_time;
  double last_time_elapsed;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer;
  OccupancyGridSubPtr map_subscriber;

  // subscribe to trajectory
  MarkerSubPtr trajectory_sub;
  void trajectoryCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

  // trajectory data storage
  std::vector<std::pair<double, double>> trajectory_points; // x, y coordinates
};

#endif // EXP_STATISTICS_H
