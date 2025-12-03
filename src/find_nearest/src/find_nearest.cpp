#include "../include/find_nearest.h"

FindNearest::FindNearest() : Node("find_nearest"), visible_received(false) {
  // initialize subscribers
  visible_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/visible_map", 10,
      std::bind(&FindNearest::visibleCallback, this, std::placeholders::_1));

  std::string robot = "av1";

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot + "/odom", 10,
      std::bind(&FindNearest::odomCallback, this, std::placeholders::_1));

  // initialize publishers
  frontier_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/frontiers_cloud", 10);
  raw_frontier_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/raw_frontiers_cloud", 10);
  path_pub = this->create_publisher<nav_msgs::msg::Path>(
      "/" + robot + "/planned_path", 10);

  // initialize neighbor directions for 8-connected grid
  neighbor_dirs = {{1, 0}, {-1, 0}, {0, 1},   {0, -1},
                   {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

  // initialize timer - runs every 1 second
  timer = this->create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&FindNearest::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "FindNearest node initialized");
}

void FindNearest::timerCallback() {
  std::lock_guard<std::mutex> lock(map_mutex);
  if (!visible_received) {
    RCLCPP_INFO(this->get_logger(), "waiting for map data...");
    return;
  }

  // 1. find and cluster frontiers
  raw_frontiers = findFrontiers();
  frontiers = clusterFrontiers(raw_frontiers, 6); // cluster size of 6

  // 2. move frontiers away from obstacles
  moveFrontiersAwayFromObstacle();

  // 3. publish frontier visualization
  publishFrontierCloud();
  publishRawFrontierCloud();

  // 4. plan path if needed
  if (replan_flag) {
    assignNearestFrontier();
  }
}

void FindNearest::visibleCallback(const OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(map_mutex);

  size_t expected_size = msg->info.width * msg->info.height;
  if (msg->data.size() != expected_size) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Map data size mismatch! Width: %d, Height: %d, Data Size: %zu",
        msg->info.width, msg->info.height, msg->data.size());
    return;
  }

  visible = *msg;
  visible_cost = generateCostmap(visible);

  // update map parameters
  width = visible.info.width;
  height = visible.info.height;
  resolution = visible.info.resolution;
  origin_x = visible.info.origin.position.x;
  origin_y = visible.info.origin.position.y;

  visible_received = true;

  RCLCPP_DEBUG(this->get_logger(), "Map received: %dx%d, resolution: %.2f",
               width, height, resolution);
}

void FindNearest::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null odometry message");
    return;
  }

  std::lock_guard<std::mutex> lock(map_mutex);
  current_pose = msg->pose.pose;

  // check if we need to replan
  if (!replan_flag) {
    bool arrived = checkArriveTarget();
    bool traveled = checkTravelFarEnough();

    if (arrived || traveled) {
      RCLCPP_INFO(this->get_logger(),
                  "Replanning triggered - arrived: %s, traveled: %s",
                  arrived ? "true" : "false", traveled ? "true" : "false");
      // if arrived, mark the target frontier as visited so it is not
      // reselected repeatedly in subsequent replans.
      if (arrived) {
        visited_frontiers.insert(target_frontier);
        // reset the target since it has been visited
        target_frontier = {0, 0};
      }
      replan_flag = true;
    }
  }

  // assign new frontier if needed
  if (replan_flag) {
    assignNearestFrontier();
  }
}

OccupancyGrid FindNearest::generateCostmap(const OccupancyGrid &original_map) {
  OccupancyGrid new_map = original_map;
  new_map.data = original_map.data;

  if (original_map.info.width <= 0 || original_map.info.height <= 0 ||
      original_map.data.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid input map: empty or invalid dimensions");
    return new_map;
  }

  // inflate obstacles by 1 cell
  for (int y = 0; y < static_cast<int>(original_map.info.height); y++) {
    for (int x = 0; x < static_cast<int>(original_map.info.width); x++) {
      int index = y * original_map.info.width + x;
      if (original_map.data[index] == 100) {
        // mark 8-connected neighbors as high cost
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
                // high cost but not blocked
                new_map.data[new_index] = 75;
              }
            }
          }
        }
      }
    }
  }

  return new_map;
}

std::vector<Point2D> FindNearest::findFrontiers() {
  std::vector<Point2D> frontiers;

  if (!visible_received) {
    return frontiers;
  }

  int width = visible.info.width;
  int height = visible.info.height;

  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      if (isFrontier(visible.data, x, y)) {
        frontiers.push_back({x, y});
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Found %zu raw frontiers", frontiers.size());
  return frontiers;
}

std::vector<Point2D>
FindNearest::clusterFrontiers(const std::vector<Point2D> &raw_frontiers,
                              int cluster_size) {
  std::map<Point2D, std::vector<Point2D>> grid_clusters;

  // assign frontiers to grid cells
  for (const auto &pt : raw_frontiers) {
    int grid_x = pt.x / cluster_size;
    int grid_y = pt.y / cluster_size;
    Point2D grid_pt{grid_x, grid_y};
    grid_clusters[grid_pt].push_back(pt);
  }

  std::vector<Point2D> clustered_frontiers;
  for (const auto &[grid_key, points_in_cluster] : grid_clusters) {
    if (points_in_cluster.empty())
      continue;

    // calculate cluster center
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &pt : points_in_cluster) {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    double center_x = sum_x / points_in_cluster.size();
    double center_y = sum_y / points_in_cluster.size();

    // find point closest to cluster center
    double min_dist = std::numeric_limits<double>::max();
    Point2D best_point = points_in_cluster[0];

    for (const auto &pt : points_in_cluster) {
      double dx = pt.x - center_x;
      double dy = pt.y - center_y;
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        best_point = pt;
      }
    }

    clustered_frontiers.push_back(best_point);
  }

  RCLCPP_INFO(this->get_logger(), "Clustered into %zu frontiers",
              clustered_frontiers.size());
  return clustered_frontiers;
}

void FindNearest::moveFrontiersAwayFromObstacle() {
  std::vector<Point2D> new_frontiers;

  for (const auto &pt : frontiers) {
    int x = pt.x;
    int y = pt.y;
    int obstacle_count = 0;
    Point2D shift_direction{0, 0};

    // check 8 neighbors for obstacles
    for (const auto &dir : neighbor_dirs) {
      int nx = x + dir.first;
      int ny = y + dir.second;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      int idx = ny * width + nx;
      if (idx >= 0 && idx < static_cast<int>(visible.data.size()) &&
          visible.data[idx] >= 50) { // obstacle threshold
        obstacle_count++;
        shift_direction.x -= dir.first;
        shift_direction.y -= dir.second;
      }
    }

    // if surrounded by 3+ obstacles, shift away
    if (obstacle_count >= 3) {
      double magnitude = std::hypot(shift_direction.x, shift_direction.y);
      if (magnitude > 0) {
        shift_direction.x = std::round(shift_direction.x / magnitude);
        shift_direction.y = std::round(shift_direction.y / magnitude);
      }

      // find valid position by shifting
      int new_x = x, new_y = y;
      bool valid_position = false;

      for (int step = 1; step <= 5; ++step) {
        new_x = x + step * shift_direction.x;
        new_y = y + step * shift_direction.y;

        if (new_x < 0 || new_x >= width || new_y < 0 || new_y >= height) {
          break;
        }

        int new_idx = new_y * width + new_x;
        if (new_idx >= 0 && new_idx < static_cast<int>(visible.data.size()) &&
            visible.data[new_idx] == 0) {
          valid_position = true;
          break;
        }
      }

      if (valid_position) {
        new_frontiers.push_back({new_x, new_y});
      } else {
        // new_frontiers.push_back(pt);
        RCLCPP_DEBUG(
            this->get_logger(),
            "Could not move frontier (%d,%d) away from obstacles. Discard.", x,
            y);
      }
    } else {
      new_frontiers.push_back(pt);
    }
  }
  frontiers = new_frontiers;
}

bool FindNearest::aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                              std::vector<Point2D> &path) {
  if (!visible_received) {
    RCLCPP_WARN(this->get_logger(), "Map data not available for path planning");
    return false;
  }

  // boundary checks
  if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
      goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
    RCLCPP_WARN(this->get_logger(), "Start or goal out of bounds");
    return false;
  }

  // obstacle checks
  int start_idx = start_y * width + start_x;
  int goal_idx = goal_y * width + goal_x;
  if (visible.data[start_idx] >= 50 || visible.data[goal_idx] >= 50) {
    RCLCPP_WARN(this->get_logger(), "Start or goal in obstacle");
    return false;
  }

  std::priority_queue<Node2D, std::vector<Node2D>, NodeComparator> open;
  std::unordered_set<int64_t> closed;
  std::unordered_map<int64_t, std::shared_ptr<Node2D>> nodes;

  // initialize start node
  Node2D start(start_x, start_y, 0.0);
  start.h = std::hypot(start_x - goal_x, start_y - goal_y);
  start.f = start.g + start.h;

  int64_t start_key = start.to_key(width);
  nodes[start_key] = std::make_shared<Node2D>(start);
  open.push(start);

  while (!open.empty()) {
    Node2D current = open.top();
    open.pop();
    int64_t current_key = current.to_key(width);

    if (closed.find(current_key) != closed.end()) {
      continue;
    }
    closed.insert(current_key);

    // goal reached
    if (current.x == goal_x && current.y == goal_y) {
      std::vector<Point2D> reversed_path;
      std::shared_ptr<Node2D> node = nodes[current_key];

      while (node) {
        reversed_path.push_back({node->x, node->y});
        node = node->parent;
      }

      std::reverse(reversed_path.begin(), reversed_path.end());
      path = reversed_path;
      return true;
    }

    // explore neighbors
    for (const auto &dir : neighbor_dirs) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      int idx = ny * width + nx;

      if (idx < 0 || idx >= static_cast<int>(visible.data.size()) ||
          idx >= static_cast<int>(visible_cost.data.size())) {
        continue;
      }

      if (visible_cost.data[idx] >= 80 || visible.data[idx] < 0) {
        continue; // high cost or unknown
      }

      // diagonal movement validation
      bool is_diagonal = (dir.first != 0 && dir.second != 0);
      if (is_diagonal) {
        int n1x = current.x + dir.first;
        int n1y = current.y;
        int n2x = current.x;
        int n2y = current.y + dir.second;

        if ((n1x < 0 || n1x >= width || n1y < 0 || n1y >= height ||
             visible.data[n1y * width + n1x] >= 50) ||
            (n2x < 0 || n2x >= width || n2y < 0 || n2y >= height ||
             visible.data[n2y * width + n2x] >= 50)) {
          // can't move diagonally through corners
          continue;
        }
      }

      int64_t key = ny * width + nx;
      if (closed.find(key) != closed.end()) {
        continue;
      }

      // calculate cost
      double step_cost = is_diagonal ? 1.414 : 1.0;
      if (visible_cost.data[idx] > 50) {
        // penalty for high-cost areas
        step_cost += 20.0;
      }

      Node2D neighbor(nx, ny, current.g + step_cost);
      neighbor.h = std::hypot(nx - goal_x, ny - goal_y);
      neighbor.f = neighbor.g + neighbor.h;
      neighbor.parent = nodes[current_key];

      if (nodes.find(key) == nodes.end() || neighbor.g < nodes[key]->g) {
        nodes[key] = std::make_shared<Node2D>(neighbor);
        open.push(neighbor);
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "No path found from (%d,%d) to (%d,%d)",
              start_x, start_y, goal_x, goal_y);
  return false;
}

void FindNearest::assignNearestFrontier() {
  if (frontiers.empty()) {
    RCLCPP_WARN(this->get_logger(), "No frontiers available for assignment");
    return;
  }

  // get current robot position in grid coordinates
  int robot_x, robot_y;
  worldToGrid(current_pose, robot_x, robot_y);

  // find nearest reachable frontier
  std::map<double, Point2D> distance_map;

  for (const auto &frontier : frontiers) {
    // skip visited frontiers
    if (visited_frontiers.find(frontier) != visited_frontiers.end())
      continue;
    std::vector<Point2D> path;
    if (aStarSearch(robot_x, robot_y, frontier.x, frontier.y, path)) {
      // if path size <= 1, we are already at this frontier - mark visited and
      // skip
      if (path.size() <= 1) {
        visited_frontiers.insert(frontier);
        continue;
      }
      // calculate path length
      double path_length = 0.0;
      for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        path_length += (dx != 0 && dy != 0) ? 1.414 : 1.0;
      }
      distance_map[path_length] = frontier;
    }
  }

  if (distance_map.empty()) {
    RCLCPP_WARN(this->get_logger(), "No reachable frontiers found");
    return;
  }

  // assign nearest frontier
  target_frontier = distance_map.begin()->second;

  // recompute and publish path
  std::vector<Point2D> path;
  if (aStarSearch(robot_x, robot_y, target_frontier.x, target_frontier.y,
                  path)) {
    publishPath(path);
    start_point = path.front();
    replan_flag = false;

    RCLCPP_INFO(this->get_logger(),
                "Assigned frontier (%d,%d) with path length %zu",
                target_frontier.x, target_frontier.y, path.size());
  }
}

bool FindNearest::isFrontier(const std::vector<signed char> &map_data, int x,
                             int y) {
  if (x <= 0 || x >= width - 1 || y <= 0 || y >= height - 1) {
    return false;
  }

  int idx = y * width + x;
  if (idx < 0 || idx >= static_cast<int>(map_data.size()) ||
      map_data[idx] != 0) {
    return false; // not free space
  }

  // count unknown neighbors
  int unknown_count = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dy == 0)
        continue;

      int nx = x + dx;
      int ny = y + dy;
      int nidx = ny * width + nx;

      if (nidx >= 0 && nidx < static_cast<int>(map_data.size()) &&
          map_data[nidx] == -1) {
        unknown_count++;
      }
    }
  }
  // need at least 2 unknown neighbors
  return unknown_count >= 2;
}

bool FindNearest::checkArriveTarget() {
  int robot_x, robot_y;
  worldToGrid(current_pose, robot_x, robot_y);

  double distance =
      std::hypot(robot_x - target_frontier.x, robot_y - target_frontier.y);

  return distance <= distance_threshold;
}

bool FindNearest::checkTravelFarEnough() {
  if (target_frontier.x == 0 && target_frontier.y == 0) {
    return false;
  }

  int robot_x, robot_y;
  worldToGrid(current_pose, robot_x, robot_y);

  double distance =
      std::hypot(robot_x - start_point.x, robot_y - start_point.y);

  // RCLCPP_DEBUG(
  //     this->get_logger(),
  //     "Robot at (%d,%d), start at (%d,%d), traveled: %.2f, threshold: %.2f",
  //     robot_x, robot_y, start_point.x, start_point.y, distance,
  //     travel_distance_threshold);

  return distance >= travel_distance_threshold;
}

void FindNearest::worldToGrid(const Pose &pose, int &grid_x, int &grid_y) {
  double world_x = pose.position.x;
  double world_y = pose.position.y;

  grid_x = std::floor((world_x - origin_x) / resolution);
  grid_y = std::floor((world_y - origin_y) / resolution);
}

void FindNearest::publishFrontierCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = visible.header;
  cloud.header.stamp = this->now();
  cloud.height = 1;
  cloud.width = frontiers.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  // set point cloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3, "x", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32);

  // fill cloud data
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  double robot_height = current_pose.position.z;

  for (const auto &pt : frontiers) {
    *iter_x = pt.x * resolution + origin_x + resolution / 2.0;
    *iter_y = pt.y * resolution + origin_y + resolution / 2.0;
    *iter_z = static_cast<float>(robot_height);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  frontier_pub->publish(cloud);
}

void FindNearest::publishRawFrontierCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = visible.header;
  cloud.header.stamp = this->now();
  cloud.height = 1;
  cloud.width = raw_frontiers.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  // set point cloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3, "x", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32);

  // fill cloud data
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  double robot_height = current_pose.position.z;

  for (const auto &pt : raw_frontiers) {
    *iter_x = pt.x * resolution + origin_x + resolution / 2.0;
    *iter_y = pt.y * resolution + origin_y + resolution / 2.0;
    *iter_z = static_cast<float>(robot_height);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  raw_frontier_pub->publish(cloud);
}

void FindNearest::publishPath(const std::vector<Point2D> &path) {
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = visible.header.frame_id;
  ros_path.header.stamp = this->now();

  double robot_height = current_pose.position.z;

  for (const auto &pt : path) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = ros_path.header;
    pose_stamped.pose.position.x = origin_x + (pt.x + 0.5) * resolution;
    pose_stamped.pose.position.y = origin_y + (pt.y + 0.5) * resolution;
    pose_stamped.pose.position.z = robot_height;
    pose_stamped.pose.orientation.w = 1.0;
    ros_path.poses.push_back(pose_stamped);
  }

  path_pub->publish(ros_path);
}
