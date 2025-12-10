#include "../include/path_selector.h"
#include <limits>

// Initialize static member for 8-connected movement directions with
// orientations
const std::vector<std::pair<std::pair<int, int>, double>>
    PathSelector::neighbor_dirs = {
        {{0, 1}, M_PI / 2},       // (0, 1) - up
        {{1, 0}, 0.0},            // (1, 0) - right
        {{0, -1}, -M_PI / 2},     // (0, -1) - down
        {{-1, 0}, M_PI},          // (-1, 0) - left
        {{1, 1}, M_PI / 4},       // (1, 1) - up-right diagonal
        {{-1, 1}, 3 * M_PI / 4},  // (-1, 1) - up-left diagonal
        {{1, -1}, -M_PI / 4},     // (1, -1) - down-right diagonal
        {{-1, -1}, -3 * M_PI / 4} // (-1, -1) - down-left diagonal
};

PathSelector::PathSelector()
    : Node("path_selector"), odom_received(false), visible_received(false),
      curr_index(1) {
  // subscribe to local plan result
  std::string local_topic = "/local_planned_path";
  local_plan_sub = this->create_subscription<prob_msgs::msg::PathPoseArray>(
      local_topic, 10,
      std::bind(&PathSelector::localPathCallback, this, std::placeholders::_1));

  // subscribe to global plan result
  std::string global_topic = "/global_planned_path";
  global_plan_sub = this->create_subscription<prob_msgs::msg::PathPoseArray>(
      global_topic, 10,
      std::bind(&PathSelector::globalPathCallback, this,
                std::placeholders::_1));

  // subscribe to odom
  this->declare_parameter<std::string>("robot_name", "av1");
  std::string robot = this->get_parameter("robot_name").as_string();
  std::string odom_topic = "/" + robot + "/odom";
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&PathSelector::odomCallback, this, std::placeholders::_1));

  // subscribe to map
  visible_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/visible_map", 10,
      std::bind(&PathSelector::visibleCallback, this, std::placeholders::_1));

  // publish path
  std::string path_topic = "/" + robot + "/planned_path";
  path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
  solution_pub = this->create_publisher<nav_msgs::msg::Path>(
      "/" + robot + "/solution_path", 10);

  this->declare_parameter<double>("height", 0.0);
  height = this->get_parameter("height").as_double();

  this->declare_parameter<double>("waypoint_tolerance", 0.5);
  waypoint_tolerance = this->get_parameter("waypoint_tolerance").as_double();

  this->declare_parameter<double>("direction_change_penalty", 1.0);
  direction_change_penalty =
      this->get_parameter("direction_change_penalty").as_double();
}

void PathSelector::localPathCallback(
    const prob_msgs::msg::PathPoseArray::SharedPtr msg) {
  if (msg->poses.empty() && global_res.poses.empty()) {
    local_res.poses.clear();
    exe_path.poses.clear();
    curr_index = 0;

    // publish empty path
    auto empty_path = std::make_unique<nav_msgs::msg::Path>();
    empty_path->header.stamp = this->get_clock()->now();
    empty_path->header.frame_id = "map";
    path_pub->publish(std::move(empty_path));

    RCLCPP_WARN(this->get_logger(),
                "Published empty path - no local or global paths available");
    return;
  }
  if (msg->poses.empty()) {
    local_res.poses.clear();
    return;
  }
  if (msg->poses == exe_path.poses) {
    return;
  }
  local_res = *msg;
  // update exe_path
  exe_path = local_res;
  curr_index = 1;

  // Log when starting to follow a new local path
  if (!exe_path.poses.empty() &&
      curr_index < static_cast<int>(exe_path.poses.size())) {
    const auto &first_waypoint = exe_path.poses[curr_index].pose;
    RCLCPP_INFO(this->get_logger(),
                "We are now heading for waypoint %d, location (%.2f, %.2f)",
                curr_index, first_waypoint.position.x,
                first_waypoint.position.y);
  }
}

void PathSelector::globalPathCallback(
    const prob_msgs::msg::PathPoseArray::SharedPtr msg) {
  if (msg->poses.empty() && local_res.poses.empty()) {
    global_res.poses.clear();
    exe_path.poses.clear();
    curr_index = 0;
  }
  if (msg->poses.empty()) {
    return;
  }
  if (msg->poses == exe_path.poses) {
    return;
  }

  // Simple logic: directly use new path as received, start from first waypoint
  if (!msg->poses.empty()) {
    curr_index = 1; // start from first waypoint
    RCLCPP_INFO(
        this->get_logger(),
        "Received new global path with %zu poses, starting from waypoint %d",
        msg->poses.size(), curr_index);
  }

  global_res = *msg;
  // update exe_path
  exe_path = global_res;

  // log when starting to follow a new global path
  if (!exe_path.poses.empty() &&
      curr_index < static_cast<int>(exe_path.poses.size())) {
    const auto &first_waypoint = exe_path.poses[curr_index].pose;
    RCLCPP_INFO(this->get_logger(),
                "We are now heading for waypoint %d, location (%.2f, %.2f)",
                curr_index, first_waypoint.position.x,
                first_waypoint.position.y);
  }
}

void PathSelector::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg || exe_path.poses.empty()) {
    return;
  }

  // store current pose
  current_pose = msg->pose.pose;
  odom_received = true;

  // check if curr_index is valid
  if (curr_index < 0 || curr_index >= static_cast<int>(exe_path.poses.size())) {
    curr_index = std::min(1, static_cast<int>(exe_path.poses.size()) - 1);
  }

  // update curr_index based on distance to current waypoint
  while (curr_index < static_cast<int>(exe_path.poses.size())) {
    const auto &current_waypoint = exe_path.poses[curr_index].pose;

    // calculate distance from robot to current waypoint
    double dx = current_pose.position.x - current_waypoint.position.x;
    double dy = current_pose.position.y - current_waypoint.position.y;
    double distance_to_waypoint = std::hypot(dx, dy);

    // check if robot is close enough to current waypoint
    if (distance_to_waypoint <= waypoint_tolerance) {
      // robot has reached current waypoint, advance to next one
      curr_index++;
      if (DEBUG_MODE) {
        RCLCPP_INFO(
            this->get_logger(),
            "Reached waypoint %d, advancing to waypoint %d (distance: %.3f)",
            curr_index - 1, curr_index, distance_to_waypoint);
      }

      // log when advancing to next waypoint
      if (curr_index < static_cast<int>(exe_path.poses.size())) {
        const auto &next_waypoint = exe_path.poses[curr_index].pose;
        RCLCPP_INFO(this->get_logger(),
                    "We are now heading for waypoint %d, location (%.2f, %.2f)",
                    curr_index, next_waypoint.position.x,
                    next_waypoint.position.y);
      }

      // check if we've reached the end of the path
      if (curr_index >= static_cast<int>(exe_path.poses.size())) {
        if (DEBUG_MODE) {
          RCLCPP_INFO(this->get_logger(), "Reached final waypoint of the path");
        }
        // publish empty path or keep curr_index at end?
        // for now, keep at end and no more A* planning
        return;
      }
    } else {
      // robot is not close enough, stop advancing
      break;
    }
  }

  // get current target pose
  const auto &target_pose = exe_path.poses[curr_index].pose;

  // convert robot position to grid coordinates (continuous, no snapping)
  double robot_x_cont = current_pose.position.x;
  double robot_y_cont = current_pose.position.y;
  int robot_x =
      static_cast<int>((robot_x_cont - visible.info.origin.position.x) /
                       visible.info.resolution);
  int robot_y =
      static_cast<int>((robot_y_cont - visible.info.origin.position.y) /
                       visible.info.resolution);

  // convert target position to grid coordinates (snapped to grid)
  double snapped_goal_x =
      std::round(target_pose.position.x / visible.info.resolution) *
      visible.info.resolution;
  double snapped_goal_y =
      std::round(target_pose.position.y / visible.info.resolution) *
      visible.info.resolution;
  int goal_x =
      static_cast<int>((snapped_goal_x - visible.info.origin.position.x) /
                       visible.info.resolution);
  int goal_y =
      static_cast<int>((snapped_goal_y - visible.info.origin.position.y) /
                       visible.info.resolution);

  // validate robot position - if in obstacle, search nearby for free position
  if (robot_x >= 0 && robot_x < static_cast<int>(visible.info.width) &&
      robot_y >= 0 && robot_y < static_cast<int>(visible.info.height)) {
    int robot_idx = robot_y * visible.info.width + robot_x;
    if (visible.data[robot_idx] >= 80) {
      // search for nearby free position
      bool found_free = false;
      for (int radius = 1; radius <= 3 && !found_free; radius++) {
        for (int dx = -radius; dx <= radius && !found_free; dx++) {
          for (int dy = -radius; dy <= radius && !found_free; dy++) {
            int nx = robot_x + dx;
            int ny = robot_y + dy;
            if (nx >= 0 && nx < static_cast<int>(visible.info.width) &&
                ny >= 0 && ny < static_cast<int>(visible.info.height)) {
              int nidx = ny * visible.info.width + nx;
              if (visible.data[nidx] < 50) {
                robot_x = nx;
                robot_y = ny;
                found_free = true;
              }
            }
          }
        }
      }
      if (!found_free) {
        RCLCPP_WARN(
            this->get_logger(),
            "Robot is in obstacle and couldn't find free position nearby");
        return;
      }
    }
  }

  // perform A* search from robot to target
  std::vector<Point2D> a_star_path;
  if (aStarSearch(robot_x, robot_y, goal_x, goal_y, a_star_path)) {

    // convert A* path to nav_msgs::msg::Path and publish
    nav_msgs::msg::Path output_path;
    output_path.header.stamp = this->get_clock()->now();
    output_path.header.frame_id = "map";

    // add robot's current position as first point
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header = output_path.header;
    robot_pose.pose = current_pose;
    // output_path.poses.push_back(robot_pose);

    // add A* path points
    for (size_t i = 0; i < a_star_path.size(); i++) {
      geometry_msgs::msg::PoseStamped path_pose;
      path_pose.header = output_path.header;
      path_pose.pose.position.x =
          visible.info.origin.position.x +
          (a_star_path[i].x + 0.5) * visible.info.resolution;
      path_pose.pose.position.y =
          visible.info.origin.position.y +
          (a_star_path[i].y + 0.5) * visible.info.resolution;
      path_pose.pose.position.z = height;
      path_pose.pose.orientation.x = 0.0;
      path_pose.pose.orientation.y = 0.0;
      path_pose.pose.orientation.z = 0.0;
      path_pose.pose.orientation.w = 1.0;
      output_path.poses.push_back(path_pose);
    }

    // publish the constructed path
    path_pub->publish(output_path);
    nav_msgs::msg::Path solution_path = convertPathPoseArrayToPath(exe_path);
    solution_pub->publish(solution_path);

    if (DEBUG_MODE) {
      RCLCPP_INFO(this->get_logger(),
                  "Published A* path with %zu poses from robot to waypoint %d",
                  output_path.poses.size(), curr_index);
    }
  } else {
    if (DEBUG_MODE) {
      RCLCPP_WARN(this->get_logger(),
                  "A* search failed from robot (%d, %d) to target (%d, %d)",
                  robot_x, robot_y, goal_x, goal_y);
    }
  }
}

void PathSelector::visibleCallback(
    nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!msg) {
    return;
  }
  visible = *msg;
  visible_cost = generateCostmap(visible);
  visible_received = true;
}

OccupancyGrid PathSelector::generateCostmap(const OccupancyGrid &original_map) {
  OccupancyGrid new_map = original_map;
  new_map.data = original_map.data;

  if (original_map.info.width <= 0 || original_map.info.height <= 0 ||
      original_map.data.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid input map: empty or invalid dimensions");
    return new_map;
  }

  // Inflate obstacles by 1 cell
  for (int y = 0; y < static_cast<int>(original_map.info.height); y++) {
    for (int x = 0; x < static_cast<int>(original_map.info.width); x++) {
      int index = y * original_map.info.width + x;
      if (original_map.data[index] == 100) { // occupied cell
        // Mark 8-connected neighbors as high cost
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            int new_y = y + dy;
            int new_x = x + dx;
            if (new_x >= 0 &&
                new_x < static_cast<int>(original_map.info.width) &&
                new_y >= 0 &&
                new_y < static_cast<int>(original_map.info.height)) {
              int new_index = new_y * original_map.info.width + new_x;
              if (new_map.data[new_index] != 100 &&
                  new_map.data[new_index] == 0) {
                new_map.data[new_index] = 75; // high cost but not blocked
              }
            }
          }
        }
      }
    }
  }

  return new_map;
}

bool PathSelector::aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                               std::vector<Point2D> &path) {
  if (!visible_received) {
    RCLCPP_WARN(this->get_logger(), "Map data not available for path planning");
    return false;
  }

  int width = visible.info.width;
  int height = visible.info.height;

  if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
      goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
    RCLCPP_INFO(this->get_logger(),
                "A* failed: Start/Goal out of bounds - Start(%d,%d) "
                "Goal(%d,%d) Map(%dx%d)",
                start_x, start_y, goal_x, goal_y, width, height);
    return false;
  }

  int start_idx = start_y * width + start_x;
  int goal_idx = goal_y * width + goal_x;
  // Enhanced obstacle detection with detailed logging
  bool start_in_obstacle =
      (visible.data[start_idx] >= 50) ||
      (visible.data[start_idx] == -1 && visible.data[start_idx] >= 50);
  bool goal_in_obstacle =
      (visible.data[goal_idx] >= 50) ||
      (visible.data[goal_idx] == -1 && visible.data[goal_idx] >= 50);

  if (start_in_obstacle || goal_in_obstacle) {
    RCLCPP_WARN(this->get_logger(),
                "Start (%d,%d) in obstacle: vis=%d pred=%d | Goal (%d,%d) in "
                "obstacle: vis=%d pred=%d",
                start_x, start_y, visible.data[start_idx],
                visible.data[start_idx], goal_x, goal_y, visible.data[goal_idx],
                visible.data[goal_idx]);

    // Convert to world coordinates for debugging
    double start_world_x = visible.info.origin.position.x +
                           (start_x + 0.5) * visible.info.resolution;
    double start_world_y = visible.info.origin.position.y +
                           (start_y + 0.5) * visible.info.resolution;
    double goal_world_x = visible.info.origin.position.x +
                          (goal_x + 0.5) * visible.info.resolution;
    double goal_world_y = visible.info.origin.position.y +
                          (goal_y + 0.5) * visible.info.resolution;

    RCLCPP_WARN(this->get_logger(),
                "World coords - Start: (%.2f,%.2f) Goal: (%.2f,%.2f)",
                start_world_x, start_world_y, goal_world_x, goal_world_y);

    return false;
  }
  std::priority_queue<Node2D, std::vector<Node2D>, NodeComparator> open;
  std::unordered_set<int64_t> closed;
  std::unordered_map<int64_t, std::shared_ptr<Node2D>> nodes;
  std::unordered_map<int64_t, double> gtable;

  Node2D start(start_x, start_y, 0.0);
  start.h = std::hypot(start_x - goal_x, start_y - goal_y);
  start.f = start.g + start.h;
  int64_t start_key = start.to_key(width);
  nodes[start_key] = std::make_shared<Node2D>(start);
  gtable[start_key] = start.g;
  open.push(start);

  while (!open.empty()) {
    Node2D current = open.top();
    open.pop();
    int64_t current_key = current.to_key(width);

    if (closed.find(current_key) != closed.end()) {
      continue;
    }
    closed.insert(current_key);

    if (current.x == goal_x && current.y == goal_y) {
      std::vector<Point2D> rev;
      std::shared_ptr<Node2D> node = nodes[current_key];
      while (node) {
        rev.push_back({node->x, node->y});
        node = node->parent;
      }
      std::reverse(rev.begin(), rev.end());
      path = rev;
      return true;
    }

    for (const auto &[dir, ort] : neighbor_dirs) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height)
        continue;

      int idx = ny * width + nx;

      int8_t pred_val = visible.data[idx];
      int8_t vis_val = visible.data[idx];

      // More permissive traversal: allow if visible is free OR visible is not
      // too occupied
      bool is_traversable = false;
      if (vis_val == 0) {
        is_traversable = true; // Free or unknown in visible map
      } else if (vis_val == -1 && pred_val == 0) {
        is_traversable = true; // Low occupancy in visible map
      }

      if (!is_traversable) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Cell blocked: (%d,%d) visible=%d visible=%d", nx, ny,
                     vis_val, pred_val);
        continue;
      }

      // diagonal check
      bool is_diagonal = (dir.first != 0 && dir.second != 0);
      bool can_move = true;
      if (is_diagonal) {
        int dx = dir.first, dy = dir.second;
        int n1x = current.x + dx, n1y = current.y;
        int n2x = current.x, n2y = current.y + dy;
        if (!(n1x >= 0 && n1x < width && n1y >= 0 && n1y < height &&
              (visible.data[n1y * width + n1x] == 0 ||
               (visible.data[n1y * width + n1x] == -1 &&
                visible.data[n1y * width + n1x] < 50))))
          can_move = false;
        if (!(n2x >= 0 && n2x < width && n2y >= 0 && n2y < height &&
              (visible.data[n2y * width + n2x] == 0 ||
               (visible.data[n2y * width + n2x] == -1 &&
                visible.data[n2y * width + n2x] < 50))))
          can_move = false;
      }
      if (!can_move)
        continue;

      int64_t neighbor_key = ny * width + nx;
      if (closed.find(neighbor_key) != closed.end())
        continue;

      double step = (is_diagonal ? 1.414 : 1.0);
      if (visible_cost.data[idx] > 50) {
        step += 20.0; // penalty for high-cost areas
      }

      double tentative_g = current.g + step;

      if (gtable.find(neighbor_key) == gtable.end() ||
          tentative_g < gtable[neighbor_key]) {
        gtable[neighbor_key] = tentative_g;

        Node2D neighbor(nx, ny, tentative_g);
        neighbor.h = std::hypot(nx - goal_x, ny - goal_y);
        neighbor.f = neighbor.g + neighbor.h;

        if (nodes.find(current_key) == nodes.end())
          nodes[current_key] = std::make_shared<Node2D>(current);

        neighbor.parent = nodes[current_key];
        nodes[neighbor_key] = std::make_shared<Node2D>(neighbor);

        open.push(neighbor);
      }
    }
  }
  return false;
}

double
PathSelector::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat) {
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

nav_msgs::msg::Path PathSelector::convertPathPoseArrayToPath(
    const prob_msgs::msg::PathPoseArray &path_array) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  for (const auto &path_pose : path_array.poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose = path_pose.pose;
    pose_stamped.pose.position.z = height; // Set the height parameter
    path.poses.push_back(pose_stamped);
  }

  return path;
}
