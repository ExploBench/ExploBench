#include "../include/exp_statistics.h"
#include <filesystem>

ExpStatistics::ExpStatistics() : Node("exp_statistics_node") {
  // initialize file
  this->declare_parameter<std::string>("stat_file_path",
                                       "exploration_stats.csv");
  file_path = this->get_parameter("stat_file_path").as_string();
  namespace fs = std::filesystem;
  fs::path output_path(file_path);
  fs::path parent_path = output_path.parent_path();
  if (!parent_path.empty()) {
    std::error_code ec;
    bool created = fs::create_directories(parent_path, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                   parent_path.c_str(), ec.message().c_str());
    } else if (created) {
      RCLCPP_INFO(this->get_logger(), "Created directory: %s",
                  parent_path.c_str());
    }
  }
  output_file.open(file_path, std::ios::out);
  if (!output_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s",
                 file_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Opened file: %s", file_path.c_str());
    // write header
    output_file << "time_elapsed,explored_cells,distance" << std::endl;
    output_file << std::fixed << std::setprecision(2);
  }

  // init map subscriber
  map_received = false;
  map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/visible_map", 10,
      std::bind(&ExpStatistics::mapCallback, this, std::placeholders::_1));

  // record timer
  start_time = this->now().seconds();
  last_time_elapsed = 0.0;
  timer =
      this->create_wall_timer(std::chrono::milliseconds(250),
                              std::bind(&ExpStatistics::timerCallback, this));

  // subscribe to trajectory
  std::string robot = "av1";
  trajectory_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      "/" + robot + "/vehicle_trajectory", 10,
      std::bind(&ExpStatistics::trajectoryCallback, this,
                std::placeholders::_1));
}

ExpStatistics::~ExpStatistics() {
  if (output_file.is_open()) {
    output_file.close();
    RCLCPP_INFO(this->get_logger(), "Closed file: %s", file_path.c_str());
  }
}

void ExpStatistics::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!map_received && !msg->data.empty()) {
    map_received = true;
    if (DEBUG_INFO)
      RCLCPP_INFO(this->get_logger(), "Received map of size %d x %d",
                  msg->info.width, msg->info.height);
  }
  if (map_received) {
    current_map = *msg;
  }
}

void ExpStatistics::trajectoryCallback(
    const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (msg->type != visualization_msgs::msg::Marker::LINE_STRIP ||
      msg->points.empty()) {
    return;
  }

  // Clear previous trajectory points and store new ones
  trajectory_points.clear();
  for (const auto &point : msg->points) {
    trajectory_points.emplace_back(point.x, point.y);
  }

  if (DEBUG_INFO)
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points",
                trajectory_points.size());
}

void ExpStatistics::timerCallback() {
  if (!map_received) {
    if (DEBUG_INFO) {
      RCLCPP_WARN(this->get_logger(), "Waiting for map...");
    }

    return;
  }

  int explored_cells = 0;
  // iterate through the map and to get explorate points
  for (size_t y = 0; y < current_map.info.height; y++) {
    for (size_t x = 0; x < current_map.info.width; x++) {
      int8_t cell_value = current_map.data[y * current_map.info.width + x];
      if (cell_value != -1) {
        explored_cells++;
      }
    }
  }

  // write to local file
  double current_time = this->now().seconds();
  double time_elapsed = current_time - start_time;

  // ensure time monotonicity (prevent time from going backwards in output)
  if (time_elapsed < last_time_elapsed) {
    time_elapsed = last_time_elapsed;
    return;
  } else {
    last_time_elapsed = time_elapsed;
  }

  // calculate total distance from trajectory points
  double total_distance = 0.0;
  if (trajectory_points.size() >= 2) {
    for (size_t i = 1; i < trajectory_points.size(); i++) {
      double dx = trajectory_points[i].first - trajectory_points[i - 1].first;
      double dy = trajectory_points[i].second - trajectory_points[i - 1].second;
      total_distance += std::sqrt(dx * dx + dy * dy);
    }
  }

  if (output_file.is_open()) {
    output_file << time_elapsed << "," << explored_cells << ","
                << total_distance << std::endl;
  }
}