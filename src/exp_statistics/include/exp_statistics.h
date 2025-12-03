#ifndef EXP_STATISTICS_H
#define EXP_STATISTICS_H

#include <fstream>
#include <iomanip>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

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
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
};

#endif // EXP_STATISTICS_H
