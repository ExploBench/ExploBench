#include "../include/nbvplanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

NBVPlanner::NBVPlanner() : Node("nbvplanner"), traversable_space(0, 0, 0, 0) {
  // robot name
  this->declare_parameter("robot_name", "av1");
  this->get_parameter("robot_name", robot_name);

  // map related
  map_received = false;
  map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/visible_map", 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->mapCallback(msg);
      });

  // odom related
  odom_ready = false;
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_name + "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odomCallback(msg);
      });

  // path related
  path_pub = this->create_publisher<nav_msgs::msg::Path>(
      "/" + robot_name + "/planned_path", 10);

  // planning related
  this->declare_parameter("replan_distance", 0.1);
  this->get_parameter("replan_distance", replan_distance);
  this->declare_parameter("segment_travel_limit", 1.0);
  this->get_parameter("segment_travel_limit", segment_travel_limit);

  // rrt
  rng.seed(std::random_device()());
  root = nullptr;
  best_node = nullptr;
  exploration_solved = false;
  execution_goal = Eigen::Vector3d::Zero();
  last_plan_position = Eigen::Vector3d::Zero();
  current_segment_length = 0.0;
  has_active_segment = false;

  // parameters
  // meters
  step_size = 2;
  max_iterations = 1000;
  max_nodes = 500;
  N_tol = 1000;

  // tree visualization
  tree_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/" + robot_name + "/rrt_tree", 10);

  RCLCPP_INFO(this->get_logger(), "NBVPlanner node has been initialized.");
}

NBVPlanner::~NBVPlanner() {
  // clean up RRT tree nodes
  clearTree();
}

void NBVPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_received = true;
  map = *msg;

  // init parameters
  origin_x = map.info.origin.position.x;
  origin_y = map.info.origin.position.y;
  width = map.info.width;
  height = map.info.height;
  resolution = map.info.resolution;

  generateCostmap(map, costmap);

  traversable_space.x_max = origin_x + width * resolution;
  traversable_space.y_max = origin_y + height * resolution;
  traversable_space.x_min = origin_x;
  traversable_space.y_min = origin_y;
}

void NBVPlanner::generateCostmap(const nav_msgs::msg::OccupancyGrid &map,
                                 nav_msgs::msg::OccupancyGrid &costmap) {
  if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
    RCLCPP_WARN(this->get_logger(), "Input map is empty!");
  }
  costmap = map;

  for (uint32_t y = 0; y < map.info.height; y++) {
    for (uint32_t x = 0; x < map.info.width; x++) {
      int idx = y * map.info.width + x;
      if (map.data[idx] == 100) {
        // cell at idx is occupied
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            // skip center
            if (dx == 0 && dy == 0)
              continue;
            int next_x = x + dx;
            int next_y = y + dy;
            if (next_x >= 0 && next_x < static_cast<int>(map.info.width) &&
                next_y >= 0 && next_y < static_cast<int>(map.info.height)) {
              // only check within bounds
              int nidx = next_y * map.info.width + next_x;
              if (costmap.data[nidx] == 0) {
                // set high cost for neighboring cells of occupied cell
                costmap.data[nidx] = 75;
              }
            }
          }
        }
      }
    }
  }
}

void NBVPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_position =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  odom_ready = true;

  static rclcpp::Time last_plan_time = this->now();
  static rclcpp::Time last_check_time = this->now();
  static Eigen::Vector3d last_check_pos = robot_position;

  double time_since_last_plan = (this->now() - last_plan_time).seconds();

  double dist_to_segment_goal = has_active_segment
                                    ? (execution_goal - robot_position).norm()
                                    : std::numeric_limits<double>::infinity();
  bool close_to_goal =
      has_active_segment && dist_to_segment_goal < replan_distance;

  bool path_empty = rrt_path.poses.empty();
  bool close_to_path_end =
      !path_empty && ((Eigen::Vector3d(rrt_path.poses.back().pose.position.x,
                                       rrt_path.poses.back().pose.position.y,
                                       rrt_path.poses.back().pose.position.z) -
                       robot_position)
                          .norm() < replan_distance);

  double traveled_since_plan = (robot_position - last_plan_position).norm();
  double progress_threshold =
      std::max(0.01, 0.3 * std::max(current_segment_length, replan_distance));
  bool progressed_enough = traveled_since_plan > progress_threshold;

  bool need_plan = false;

  // 1. if path is empty and 1s is gone
  if (path_empty && time_since_last_plan > 1.0) {
    need_plan = true;
    RCLCPP_INFO(this->get_logger(),
                "Path is empty and %.1fs since last plan. Triggering replan.",
                time_since_last_plan);
  }
  // 2. get close to destination
  else if ((close_to_goal || close_to_path_end) && progressed_enough) {
    need_plan = true;
    RCLCPP_INFO(this->get_logger(),
                "%f to goal, %f to path end, traveled %.2f (threshold %.2f). "
                "Triggering replan.",
                dist_to_segment_goal,
                (Eigen::Vector3d(rrt_path.poses.back().pose.position.x,
                                 rrt_path.poses.back().pose.position.y,
                                 rrt_path.poses.back().pose.position.z) -
                 robot_position)
                    .norm(),
                traveled_since_plan, progress_threshold);
  }
  // 3. stuck at some point
  else {
    double time_since_check = (this->now() - last_check_time).seconds();
    if (time_since_check > 2.0) {
      double dist = (robot_position - last_check_pos).norm();
      if (!rrt_path.poses.empty() && dist < 0.05) { // 阈值设为0.15米
        RCLCPP_WARN(this->get_logger(),
                    "Robot stuck (moved %.2fm in %.1fs). Triggering replan.",
                    dist, time_since_check);
        need_plan = true;
      }
      last_check_time = this->now();
      last_check_pos = robot_position;
    }
  }
  if (need_plan) {
    RCLCPP_INFO(this->get_logger(), "Replanning triggered.");
    planningCallback();
    last_plan_time = this->now();
  }
}

void NBVPlanner::publishPath(nav_msgs::msg::Path path) {
  path_pub->publish(path);
}

void NBVPlanner::planningCallback() {
  if (rclcpp::ok()) {
    if (exploration_solved) {
      RCLCPP_INFO(this->get_logger(),
                  "Exploration already solved; skipping further planning.");
      return;
    }
    if (map_received && odom_ready) {
      buildTree();
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot plan path: waiting for map or odometry...");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "ROS is not ok, cannot plan path.");
  }
}

void NBVPlanner::buildTree() {
  // clear previous tree
  clearTree();
  double root_gain = computeGain(robot_position);
  if (!std::isfinite(root_gain)) {
    root_gain = 0.0;
  }
  rrt_tree.push_back(new RRTNode(robot_position, root_gain, nullptr));
  best_node = rrt_tree[0];
  root = rrt_tree[0];
  execution_goal = robot_position;
  last_plan_position = robot_position;
  current_segment_length = 0.0;
  has_active_segment = false;

  if (!force_rebuild) {
    seedTreeFromRetainedBranch();
  } else {
    clearTree();
    force_rebuild = false;
  }

  // end condition
  int failed_attempts = 0;
  const int max_failed_attempts = 500;

  auto withinBudget = [this]() {
    bool best_has_gain =
        std::isfinite(this->best_node->gain) && this->best_node->gain > 0.0;
    if (best_has_gain) {
      return this->rrt_tree.size() < this->max_nodes;
    }
    return this->rrt_tree.size() < this->N_tol;
  };

  while (withinBudget()) {
    // random Sample
    Eigen::Vector3d random_point = sampleRandomPoint();
    // find Nearest Node
    RRTNode *nearest_node = nearestNode(random_point);
    // steer
    Eigen::Vector3d new_point = steer(nearest_node->position, random_point);
    // collision Check
    if (isPathFree(nearest_node->position, new_point)) {
      // compute Gain
      double gain = computeGain(new_point);
      // skip gain is negative inf
      if (std::isinf(gain) && gain < 0) {
        failed_attempts++;
        if (failed_attempts >= max_failed_attempts) {
          RCLCPP_WARN(this->get_logger(),
                      "Too many failed attempts (%d), stopping tree expansion.",
                      failed_attempts);
          break;
        }
        continue;
      }
      // add New Node
      RRTNode *new_node = new RRTNode(new_point, gain, nearest_node);
      rrt_tree.push_back(new_node);
      failed_attempts = 0;
      // update best node
      if (gain > best_node->gain) {
        best_node = new_node;
      }
    } else {
      failed_attempts++;
      if (failed_attempts >= max_failed_attempts) {
        RCLCPP_WARN(
            this->get_logger(),
            "Too many failed path checks (%d), stopping tree expansion.",
            failed_attempts);
        break;
      }
    }
  }

  publishTreeVisualization();
  publishBestNodeVisualization();
  RCLCPP_INFO(this->get_logger(),
              "RRT tree built with %zu nodes. Best gain: %.2f at (%.2f, %.2f)",
              rrt_tree.size(), best_node->gain, best_node->position.x(),
              best_node->position.y());

  bool best_has_gain = std::isfinite(best_node->gain) && best_node->gain > 0.0;

  if (!best_has_gain) {
    retained_branch.clear();
    goal = robot_position;
    execution_goal = robot_position;
    current_segment_length = 0.0;
    last_plan_position = robot_position;
    has_active_segment = false;
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    empty_path.header.stamp = this->now();
    rrt_path = empty_path;
    publishPath(rrt_path);

    if (rrt_tree.size() >= N_tol) {
      exploration_solved = true;
      RCLCPP_WARN(
          this->get_logger(),
          "Exploration considered solved (N_T reached N_tol with zero gain).");
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Best gain remained zero; retaining no branch for next iteration.");
    }
    return;
  }

  auto best_branch = extractBranch(best_node);
  goal = best_branch.back();

  std::vector<Eigen::Vector3d> branch_prefix;
  double executed_branch_length = 0.0;
  size_t next_index =
      selectExecutionPrefix(best_branch, branch_prefix, executed_branch_length);

  if (next_index < best_branch.size()) {
    retained_branch.assign(best_branch.begin() + next_index, best_branch.end());
  } else {
    retained_branch.clear();
  }

  nav_msgs::msg::Path full_path;

  if (!constructPath(goal, full_path)) {
    RCLCPP_WARN(this->get_logger(),
                "A* planner failed to build a path to best node; publishing "
                "empty path.");
    execution_goal = robot_position;
    current_segment_length = 0.0;
    last_plan_position = robot_position;
    has_active_segment = false;
    rrt_path = full_path;
    force_rebuild = true;
    publishPath(rrt_path);
    return;
  }

  double executed_path_length = 0.0;
  nav_msgs::msg::Path clipped_path =
      clipPathByDistance(full_path, segment_travel_limit, executed_path_length);

  if (clipped_path.poses.empty()) {
    clipped_path = full_path;
    executed_path_length = 0.0;
  }

  execution_goal = Eigen::Vector3d(clipped_path.poses.back().pose.position.x,
                                   clipped_path.poses.back().pose.position.y,
                                   clipped_path.poses.back().pose.position.z);
  current_segment_length = executed_path_length;
  last_plan_position = robot_position;
  has_active_segment =
      current_segment_length > 1e-3 && clipped_path.poses.size() > 1;

  rrt_path = clipped_path;
  publishPath(rrt_path);

  RCLCPP_INFO(
      this->get_logger(),
      "Executing %.2fm along A* path (branch portion trimmed %.2fm). %zu "
      "retained nodes for next plan.",
      executed_path_length, executed_branch_length, retained_branch.size());
}

void NBVPlanner::clearTree() {
  for (auto node : rrt_tree) {
    delete node;
  }
  rrt_tree.clear();
}

void NBVPlanner::seedTreeFromRetainedBranch() {
  if (retained_branch.empty() || rrt_tree.empty()) {
    return;
  }

  RRTNode *parent = rrt_tree.front();
  size_t restored_nodes = 0;

  for (const auto &point : retained_branch) {
    if (!isPathFree(parent->position, point)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Aborting retained branch reuse due to collision risk.");
      break;
    }

    double gain = computeGain(point);
    if (!std::isfinite(gain)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Aborting retained branch reuse due to invalid gain.");
      break;
    }

    auto *node = new RRTNode(point, gain, parent);
    rrt_tree.push_back(node);
    parent = node;
    restored_nodes++;

    if (gain > best_node->gain) {
      best_node = node;
    }
  }

  if (restored_nodes == retained_branch.size()) {
    retained_branch.clear();
  } else {
    retained_branch.erase(retained_branch.begin(),
                          retained_branch.begin() + restored_nodes);
  }

  if (restored_nodes > 0) {
    RCLCPP_INFO(this->get_logger(),
                "Reused %zu nodes from previous best branch.", restored_nodes);
  }
}

std::vector<Eigen::Vector3d> NBVPlanner::extractBranch(RRTNode *node) {
  std::vector<Eigen::Vector3d> branch;
  while (node != nullptr) {
    branch.push_back(node->position);
    node = node->parent;
  }
  std::reverse(branch.begin(), branch.end());
  return branch;
}

size_t
NBVPlanner::selectExecutionPrefix(const std::vector<Eigen::Vector3d> &branch,
                                  std::vector<Eigen::Vector3d> &prefix,
                                  double &prefix_length) {
  prefix.clear();
  prefix_length = 0.0;
  if (branch.empty()) {
    return 0;
  }

  prefix.push_back(branch.front());
  if (branch.size() == 1 || segment_travel_limit <= 1e-6) {
    return 1;
  }

  double remaining = segment_travel_limit;
  size_t next_idx = 1;

  while (next_idx < branch.size() && remaining > 1e-6) {
    const Eigen::Vector3d &prev = prefix.back();
    Eigen::Vector3d target = branch[next_idx];
    double seg_len = (target - prev).norm();

    if (seg_len < 1e-6) {
      prefix.push_back(target);
      next_idx++;
      continue;
    }

    if (seg_len <= remaining + 1e-6) {
      prefix.push_back(target);
      prefix_length += seg_len;
      remaining -= seg_len;
      next_idx++;
    } else {
      Eigen::Vector3d partial = prev + (remaining / seg_len) * (target - prev);
      prefix.push_back(partial);
      prefix_length += remaining;
      remaining = 0.0;
      break;
    }
  }

  return next_idx;
}

nav_msgs::msg::Path
NBVPlanner::clipPathByDistance(const nav_msgs::msg::Path &path,
                               double max_distance, double &clipped_length) {
  nav_msgs::msg::Path clipped;
  clipped.header.frame_id = "map";
  clipped.header.stamp = this->now();
  clipped_length = 0.0;

  if (path.poses.empty()) {
    return clipped;
  }

  auto poseToVector = [](const geometry_msgs::msg::Pose &pose) {
    return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  };

  clipped.poses.push_back(path.poses.front());
  if (max_distance <= 1e-6) {
    return clipped;
  }

  double remaining = std::max(0.0, max_distance);
  for (size_t i = 1; i < path.poses.size() && remaining > 1e-6; ++i) {
    Eigen::Vector3d prev = poseToVector(path.poses[i - 1].pose);
    Eigen::Vector3d curr = poseToVector(path.poses[i].pose);
    double seg_len = (curr - prev).norm();

    if (seg_len < 1e-6) {
      clipped.poses.push_back(path.poses[i]);
      continue;
    }

    if (seg_len <= remaining + 1e-6) {
      clipped.poses.push_back(path.poses[i]);
      clipped_length += seg_len;
      remaining -= seg_len;
    } else {
      Eigen::Vector3d partial = prev + (remaining / seg_len) * (curr - prev);
      geometry_msgs::msg::PoseStamped partial_pose = path.poses[i - 1];
      partial_pose.header.stamp = this->now();
      partial_pose.pose.position.x = partial.x();
      partial_pose.pose.position.y = partial.y();
      partial_pose.pose.position.z = partial.z();
      partial_pose.pose.orientation.w = 1.0;
      clipped.poses.push_back(partial_pose);
      clipped_length += remaining;
      remaining = 0.0;
      break;
    }
  }

  return clipped;
}

Eigen::Vector3d NBVPlanner::sampleRandomPoint() {
  std::uniform_real_distribution<double> dist_x(traversable_space.x_min,
                                                traversable_space.x_max);
  std::uniform_real_distribution<double> dist_y(traversable_space.y_min,
                                                traversable_space.y_max);

  // try sampling, at most 100 times
  for (int attempt = 0; attempt < 100; ++attempt) {
    double x = dist_x(rng);
    double y = dist_y(rng);
    Eigen::Vector3d point(x, y, robot_position.z());

    // check whether point is free
    int ix = int((x - origin_x) / resolution);
    int iy = int((y - origin_y) / resolution);
    if (ix >= 0 && ix < static_cast<int>(width) && iy >= 0 &&
        iy < static_cast<int>(height)) {
      int idx = iy * width + ix;
      int8_t map_value = map.data[idx];
      int8_t costmap_value = costmap.data[idx];
      if (map_value != -1 && costmap_value == 0) {
        return point;
      }
    }
  }

  std::uniform_real_distribution<double> small_dist(-step_size, step_size);
  return Eigen::Vector3d(robot_position.x() + small_dist(rng),
                         robot_position.y() + small_dist(rng),
                         robot_position.z());
}

RRTNode *NBVPlanner::nearestNode(const Eigen::Vector3d &target) {
  RRTNode *nearest = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (auto node : rrt_tree) {
    double dist = (node->position - target).norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest = node;
    }
  }
  return nearest;
}

double NBVPlanner::computeGain(const Eigen::Vector3d &point) {
  // transform to world coordinate
  int ix = int((point.x() - origin_x) / resolution);
  int iy = int((point.y() - origin_y) / resolution);

  if (ix < 0 || ix >= static_cast<int>(width) || iy < 0 ||
      iy >= static_cast<int>(height)) {
    return -std::numeric_limits<double>::infinity();
  }
  int center_idx = iy * width + ix;
  if (map.data[center_idx] == -1 || costmap.data[center_idx] >= 50) {
    return -std::numeric_limits<double>::infinity();
  }

  // compute gain
  double unmapped_cell_gain = 3;
  double free_cell_gain = 0.1;
  double occupied_cell_penalty = -1;
  int range = 2;
  double total_gain = 0;
  for (int dy = -range; dy <= range; dy++) {
    for (int dx = -(range - std::abs(dy)); dx <= (range - std::abs(dy)); dx++) {
      int next_x = ix + dx;
      int next_y = iy + dy;
      // check bounds
      if (next_x >= 0 && next_x < static_cast<int>(width) && next_y >= 0 &&
          next_y < static_cast<int>(height)) {
        int nidx = next_y * width + next_x;
        int8_t cell_value = map.data[nidx];
        if (cell_value == -1) {
          total_gain += unmapped_cell_gain;
        } else if (cell_value == 0) {
          total_gain += free_cell_gain;
        } else if (cell_value == 100) {
          total_gain += occupied_cell_penalty;
        }
      }
    }
  }
  double distance_penalty_coefficient = -0.075; // -0.05
  double distance = (point - robot_position).norm();
  return total_gain * exp(distance_penalty_coefficient * distance);
}

Eigen::Vector3d NBVPlanner::steer(const Eigen::Vector3d &from,
                                  const Eigen::Vector3d &to) {
  Eigen::Vector3d direction = to - from;
  double distance = direction.norm();
  if (distance <= step_size) {
    return to;
  } else {
    direction.normalize();
    return from + direction * step_size;
  }
}

bool NBVPlanner::isPointFree(const Eigen::Vector3d &point) {
  // change to map index
  int ix = int((point.x() - origin_x) / resolution);
  int iy = int((point.y() - origin_y) / resolution);
  // check bounds
  if (ix < 0 || ix >= static_cast<int>(width) || iy < 0 ||
      iy >= static_cast<int>(height)) {
    return false;
  }

  int idx = iy * width + ix;
  int8_t map_value = map.data[idx];
  int8_t costmap_value = costmap.data[idx];
  return (map_value != -1 && costmap_value == 0);
}

bool NBVPlanner::isPathFree(const Eigen::Vector3d &from,
                            const Eigen::Vector3d &to) {
  Eigen::Vector3d direction = to - from;
  double distance = direction.norm();

  // if distance is small enough, treat as free
  if (distance < 1e-6) {
    return true;
  }

  Eigen::Vector3d unit_dir = direction.normalized();
  double step = resolution / 2; // check at half resolution intervals
  int total_steps = static_cast<int>(distance / step);
  for (int i = 0; i <= total_steps; ++i) {
    Eigen::Vector3d point = from + unit_dir * step * i;
    if (!isPointFree(point)) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Collision at ("
                                                  << point.x() << ", "
                                                  << point.y() << ")");
      // collision detected
      return false;
    }
  }
  // path is free
  return true;
}

bool NBVPlanner::constructPath(Eigen::Vector3d goal,
                               nav_msgs::msg::Path &path) {
  // convert coordinate to map index
  int i_start_x = int((robot_position.x() - origin_x) / resolution);
  int i_start_y = int((robot_position.y() - origin_y) / resolution);
  int i_goal_x = int((goal.x() - origin_x) / resolution);
  int i_goal_y = int((goal.y() - origin_y) / resolution);

  // A* search
  RCLCPP_INFO_STREAM(this->get_logger(), "Constructing path from ("
                                             << i_start_x << ", " << i_start_y
                                             << ") to (" << i_goal_x << ", "
                                             << i_goal_y << ")");
  struct Node2D {
    int x;
    int y;
    double g; // cost from start
    double h; // heuristic to goal
    double f() const { return g + h; }
    size_t getMapIndex(int map_width) const { return y * map_width + x; }
    std::shared_ptr<Node2D> parent;

    Node2D(int x_ = 0, int y_ = 0, double g_ = 0.0, double h_ = 0.0,
           std::shared_ptr<Node2D> parent_ = nullptr)
        : x(x_), y(y_), g(g_), h(h_), parent(parent_) {}
  };

  auto heuristic = [](int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
  };

  struct Node2DComparator {
    bool operator()(const Node2D &a, const Node2D &b) const {
      return a.f() > b.f();
    }
  };

  if (!map_received) {
    RCLCPP_WARN(this->get_logger(), "Map not received yet!");
    path = nav_msgs::msg::Path();
    return false;
  }

  // check start and goal inside map bounds
  if (i_start_x < 0 || i_start_x >= static_cast<int>(width) || i_start_y < 0 ||
      i_start_y >= static_cast<int>(height) || i_goal_x < 0 ||
      i_goal_x >= static_cast<int>(width) || i_goal_y < 0 ||
      i_goal_y >= static_cast<int>(height)) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Start (" << i_start_x << ", " << i_start_y
                                 << ") or goal (" << i_goal_x << ", "
                                 << i_goal_y << ") is out of map bounds!");
    path = nav_msgs::msg::Path();
    return false;
  }

  // check start and goal are free and known (not unknown)
  int start_idx = i_start_y * width + i_start_x;
  int goal_idx = i_goal_y * width + i_goal_x;

  if (map.data[start_idx] == -1 || costmap.data[start_idx] >= 50) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Start (" << i_start_x << ", " << i_start_y
                                 << ")  is not in explored free space!");
    path = nav_msgs::msg::Path();
    return false;
  }

  if (map.data[goal_idx] == -1 || costmap.data[goal_idx] >= 50) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Goal (" << i_goal_x << ", " << i_goal_y
                                << ")  is not in explored free space!");
    path = nav_msgs::msg::Path();
    return false;
  }

  // A* search implementation
  std::priority_queue<Node2D, std::vector<Node2D>, Node2DComparator> open;
  std::unordered_set<size_t> closed;
  closed.reserve(width * height / 10);
  std::unordered_map<size_t, std::shared_ptr<Node2D>> all_nodes;
  all_nodes.reserve(width * height / 10);

  // neighbor offsets (8-connected)
  const std::vector<std::pair<int, int>> neighbor_offsets = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}};

  // initialize Start
  Node2D start_node = {i_start_x, i_start_y, 0.0};
  start_node.h = heuristic(i_start_x, i_start_y, i_goal_x, i_goal_y);
  open.push(start_node);
  all_nodes[start_node.getMapIndex(width)] =
      std::make_shared<Node2D>(start_node);

  size_t current_node_idx = 0;

  bool path_found = false;
  while (!open.empty()) {
    Node2D curr_node = open.top();
    open.pop();
    current_node_idx = curr_node.getMapIndex(width);

    // jump over if node already visited (in closed set)
    if (closed.find(current_node_idx) != closed.end()) {
      continue;
    } else {
      closed.insert(current_node_idx);
    }

    // check if goal reached
    if (curr_node.x == i_goal_x && curr_node.y == i_goal_y) {
      path_found = true;
      break;
    }

    // explore neighbors
    for (const auto &offset : neighbor_offsets) {
      int next_x = curr_node.x + offset.first;
      int next_y = curr_node.y + offset.second;

      // check bounds
      if (next_x < 0 || next_x >= static_cast<int>(width) || next_y < 0 ||
          next_y >= static_cast<int>(height)) {
        continue;
      }

      size_t neighbor_node_idx = next_y * width + next_x;

      // check if free and explored, skip if blocked or unknown
      if (map.data[neighbor_node_idx] == -1 ||
          costmap.data[neighbor_node_idx] >= 50) {
        continue;
      }

      // check diagonals for collision
      bool is_diagonal = (offset.first != 0 && offset.second != 0);
      if (is_diagonal) {
        int dx = offset.first;
        int dy = offset.second;
        int adj1_x = curr_node.x + dx;
        int adj1_y = curr_node.y;
        int adj2_x = curr_node.x;
        int adj2_y = curr_node.y + dy;
        size_t adj1_idx = adj1_y * width + adj1_x;
        size_t adj2_idx = adj2_y * width + adj2_x;

        bool can_move = true;
        if (!(adj1_x >= 0 && adj1_x < static_cast<int>(width) && adj1_y >= 0 &&
              adj1_y < static_cast<int>(height) && adj2_x >= 0 &&
              adj2_x < static_cast<int>(width) && adj2_y >= 0 &&
              adj2_y < static_cast<int>(height) &&
              costmap.data[adj1_idx] < 50 && costmap.data[adj2_idx] < 50)) {
          can_move = false;
        }

        // if cannot move diagonally, skip
        if (!can_move) {
          continue;
        }
      }

      // if neighbor already in closed set, skip
      if (closed.find(neighbor_node_idx) != closed.end()) {
        continue;
      }

      // compute costs
      double g_cost =
          curr_node.g +
          ((offset.first == 0 || offset.second == 0) ? 1.0 : std::sqrt(2.0));
      if (costmap.data[neighbor_node_idx] > 50) {
        g_cost += 20;
      }
      double h_cost = heuristic(next_x, next_y, i_goal_x, i_goal_y);

      Node2D neighbor_node = {next_x, next_y, g_cost, h_cost};

      if (all_nodes.find(current_node_idx) == all_nodes.end()) {
        // current node not in all_nodes, add it
        all_nodes[current_node_idx] = std::make_shared<Node2D>(curr_node);
      }

      neighbor_node.parent = all_nodes[current_node_idx];

      if (all_nodes.find(neighbor_node_idx) == all_nodes.end()) {
        // neighbor not in all_nodes, add it
        all_nodes[neighbor_node_idx] = std::make_shared<Node2D>(neighbor_node);
      }

      open.push(neighbor_node);
    }
  }

  // if no path found, return empty path
  if (!path_found) {
    RCLCPP_WARN(this->get_logger(), "No path found by A*!");
    path = nav_msgs::msg::Path();
    return false;
  }

  // reconstruct path
  // nav_msgs::msg::Path path;
  path.poses.clear();
  path.header.frame_id = "map";
  path.header.stamp = this->now();

  std::shared_ptr<Node2D> path_node = all_nodes[current_node_idx];
  while (path_node != nullptr) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = origin_x + (path_node->x + 0.5) * resolution;
    pose.pose.position.y = origin_y + (path_node->y + 0.5) * resolution;
    pose.pose.position.z = robot_position.z();
    pose.pose.orientation.w = 1.0; // neutral orientation
    path.poses.push_back(pose);
    path_node = path_node->parent;
  }

  // reverse poses to get from start to goal
  std::reverse(path.poses.begin(), path.poses.end());

  // ensure the start and goal are exactly robot_position and goal
  if (!path.poses.empty()) {
    path.poses.front().pose.position.x = robot_position.x();
    path.poses.front().pose.position.y = robot_position.y();
    // path.poses.back().pose.position.x = goal.x();
    // path.poses.back().pose.position.y = goal.y();
  }

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Path: (" << path.poses.front().pose.position.x << ", "
                               << path.poses.front().pose.position.y << ") to ("
                               << path.poses.back().pose.position.x << ", "
                               << path.poses.back().pose.position.y << ") with "
                               << path.poses.size() << " waypoints.");
  return true;
}

void NBVPlanner::publishTreeVisualization() {
  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = "map";
  node_marker.header.stamp = this->now();
  node_marker.ns = robot_name + "_tree_nodes";
  node_marker.id = 0;
  node_marker.type = visualization_msgs::msg::Marker::POINTS;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.scale.x = 0.25 * resolution;
  node_marker.scale.y = 0.25 * resolution;
  node_marker.color.r = 0.0;
  node_marker.color.g = 1.0;
  node_marker.color.b = 0.0;
  node_marker.color.a = 1.0;

  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = this->now();
  edge_marker.ns = robot_name + "_tree_edges";
  edge_marker.id = 0;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.scale.x = 0.1 * resolution;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 0.5;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 0.8;

  for (const auto &node : rrt_tree) {
    geometry_msgs::msg::Point p;
    p.x = node->position.x();
    p.y = node->position.y();
    p.z = node->position.z();
    node_marker.points.push_back(p);
    if (node->parent) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = node->position.x();
      p1.y = node->position.y();
      p1.z = node->position.z();
      p2.x = node->parent->position.x();
      p2.y = node->parent->position.y();
      p2.z = node->parent->position.z();
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }

  tree_pub->publish(node_marker);
  tree_pub->publish(edge_marker);
}

void NBVPlanner::publishBestNodeVisualization() {
  if (!best_node)
    return;

  visualization_msgs::msg::Marker best_node_marker;
  best_node_marker.header.frame_id = "map";
  best_node_marker.header.stamp = this->now();
  best_node_marker.ns = robot_name + "_rrt_best_node";
  best_node_marker.id = 0;
  best_node_marker.type = visualization_msgs::msg::Marker::SPHERE;
  best_node_marker.action = visualization_msgs::msg::Marker::ADD;
  best_node_marker.scale.x = 2.0 * resolution;
  best_node_marker.scale.y = 2.0 * resolution;
  best_node_marker.scale.z = 2.0 * resolution;
  best_node_marker.color.r = 1.0;
  best_node_marker.color.g = 0.0;
  best_node_marker.color.b = 1.0;
  best_node_marker.color.a = 1.0;
  best_node_marker.pose.position.x = best_node->position.x();
  best_node_marker.pose.position.y = best_node->position.y();
  best_node_marker.pose.position.z = best_node->position.z();
  best_node_marker.pose.orientation.w = 1.0;

  tree_pub->publish(best_node_marker);
}
