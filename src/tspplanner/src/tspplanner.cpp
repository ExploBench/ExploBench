#include "../include/tspplanner.h"
#include <limits>

TSPPlanner::TSPPlanner() : Node("tspplanner"), visible_received(false) {
  // initialize subscribers
  visible_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/visible_map", 10,
      std::bind(&TSPPlanner::visibleCallback, this, std::placeholders::_1));

  std::string robot = "av1";
  this->declare_parameter("robot_name", robot);
  this->get_parameter("robot_name", robot);

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot + "/odom", 10,
      std::bind(&TSPPlanner::odomCallback, this, std::placeholders::_1));

  // initialize publishers
  frontier_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/frontiers_cloud", 10);
  path_pub = this->create_publisher<nav_msgs::msg::Path>(
      "/" + robot + "/planned_path", 10);

  // initialize neighbor directions for 8-connected grid
  neighbor_dirs = {{1, 0}, {-1, 0}, {0, 1},   {0, -1},
                   {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

  // initialize timer - runs every 2 seconds
  this->declare_parameter("replan_interval", replan_interval);
  this->get_parameter("replan_interval", replan_interval);
  planning_timer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(replan_interval * 1000)),
      std::bind(&TSPPlanner::planningCallback, this));

  this->declare_parameter("cluster_size", cluster_size);
  this->get_parameter("cluster_size", cluster_size);

  RCLCPP_INFO(this->get_logger(), "TSPPlanner node initialized");
}

void TSPPlanner::visibleCallback(const OccupancyGrid::SharedPtr msg) {
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

void TSPPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null odometry message");
    return;
  }

  std::lock_guard<std::mutex> lock(map_mutex);
  current_pose = msg->pose.pose;
  worldToGrid(current_pose, robot_grid_pos.x, robot_grid_pos.y);
}

void TSPPlanner::planningCallback() {
  std::lock_guard<std::mutex> lock(map_mutex);
  if (!visible_received) {
    RCLCPP_INFO(this->get_logger(), "waiting for map data...");
    return;
  }

  // 1. find and cluster frontiers
  raw_frontiers = findFrontiers();
  frontiers = clusterFrontiers(raw_frontiers, cluster_size);

  // 2. move frontiers away from obstacles
  moveFrontiersAwayFromObstacle();

  // 3. publish frontier visualization
  publishFrontierCloud();

  // 4. if we have frontiers, solve TSP
  if (frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "No frontiers found, exploration complete?");
    return;
  }

  // 5. build cost matrix using A*
  buildCostMatrix();

  // 6. solve TSP
  if (solveTSP()) {
    // 7. build and publish path
    std::vector<Point2D> path = buildPathFromTSP();
    publishPath(path);
    RCLCPP_INFO(this->get_logger(),
                "TSP solved, path with %zu waypoints published",
                path.size());
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to solve TSP");
  }
}

OccupancyGrid TSPPlanner::generateCostmap(const OccupancyGrid &original_map) {
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

std::vector<Point2D> TSPPlanner::findFrontiers() {
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

bool TSPPlanner::isFrontier(const std::vector<signed char> &map_data, int x,
                             int y) {
  if (x <= 0 || x >= width - 1 || y <= 0 || y >= height - 1) {
    return false;
  }

  int idx = y * width + x;
  if (idx < 0 || idx >= static_cast<int>(map_data.size()) ||
      map_data[idx] != 0) {
    return false; // not free space
  }

  // count unknown neighbors (need at least 2)
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

std::vector<Point2D>
TSPPlanner::clusterFrontiers(const std::vector<Point2D> &raw_frontiers,
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

void TSPPlanner::moveFrontiersAwayFromObstacle() {
  std::vector<Point2D> new_frontiers;
  const int safety_distance = 3; // cells away from obstacles

  for (const auto &pt : frontiers) {
    int x = pt.x;
    int y = pt.y;
    
    // Check if current position is in obstacle or too close
    int idx = y * width + x;
    if (idx < 0 || idx >= static_cast<int>(visible.data.size()) ||
        visible.data[idx] >= 50) {
      // Already in obstacle, need to move
    } else {
      // Check safety distance - look for obstacles in surrounding area
      bool too_close = false;
      for (int dy = -safety_distance; dy <= safety_distance && !too_close; dy++) {
        for (int dx = -safety_distance; dx <= safety_distance && !too_close; dx++) {
          int nx = x + dx;
          int ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          
          int nidx = ny * width + nx;
          if (nidx >= 0 && nidx < static_cast<int>(visible.data.size()) &&
              visible.data[nidx] >= 50) {
            double dist = std::hypot(dx, dy);
            if (dist < safety_distance) {
              too_close = true;
            }
          }
        }
      }
      
      if (!too_close) {
        // Safe position, keep it
        new_frontiers.push_back(pt);
        continue;
      }
    }

    // Need to move away from obstacles
    Point2D shift_direction{0, 0};
    int obstacle_count = 0;

    // Check larger area for obstacles to determine shift direction
    for (int dy = -safety_distance; dy <= safety_distance; dy++) {
      for (int dx = -safety_distance; dx <= safety_distance; dx++) {
        if (dx == 0 && dy == 0) continue;
        
        int nx = x + dx;
        int ny = y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

        int nidx = ny * width + nx;
        if (nidx >= 0 && nidx < static_cast<int>(visible.data.size()) &&
            visible.data[nidx] >= 50) {
          obstacle_count++;
          // Shift away from obstacle
          double dist = std::hypot(dx, dy);
          if (dist > 0) {
            shift_direction.x -= static_cast<int>(std::round(dx / dist));
            shift_direction.y -= static_cast<int>(std::round(dy / dist));
          }
        }
      }
    }

    if (obstacle_count == 0) {
      // No obstacles nearby, keep original position
      new_frontiers.push_back(pt);
      continue;
    }

    // Normalize shift direction
    double magnitude = std::hypot(shift_direction.x, shift_direction.y);
    if (magnitude > 0) {
      shift_direction.x = static_cast<int>(std::round(shift_direction.x / magnitude));
      shift_direction.y = static_cast<int>(std::round(shift_direction.y / magnitude));
    } else {
      // Default shift direction
      shift_direction = {1, 0};
    }

    // Find valid position by shifting
    int new_x = x, new_y = y;
    bool valid_position = false;

    for (int step = 1; step <= safety_distance * 2; ++step) {
      new_x = x + step * shift_direction.x;
      new_y = y + step * shift_direction.y;

      if (new_x < 0 || new_x >= width || new_y < 0 || new_y >= height) {
        break;
      }

      int new_idx = new_y * width + new_x;
      if (new_idx < 0 || new_idx >= static_cast<int>(visible.data.size()) ||
          visible.data[new_idx] >= 50) {
        continue; // Still in obstacle
      }

      // Check if new position is safe (no obstacles within safety_distance)
      bool safe = true;
      for (int dy = -safety_distance; dy <= safety_distance && safe; dy++) {
        for (int dx = -safety_distance; dx <= safety_distance && safe; dx++) {
          int nx = new_x + dx;
          int ny = new_y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          
          int nidx = ny * width + nx;
          if (nidx >= 0 && nidx < static_cast<int>(visible.data.size()) &&
              visible.data[nidx] >= 50) {
            double dist = std::hypot(dx, dy);
            if (dist < safety_distance) {
              safe = false;
            }
          }
        }
      }

      if (safe) {
        valid_position = true;
        break;
      }
    }

    if (valid_position) {
      new_frontiers.push_back({new_x, new_y});
    } else {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Could not move frontier (%d,%d) away from obstacles. Discard.", x,
          y);
    }
  }
  frontiers = new_frontiers;
}

bool TSPPlanner::aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
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

double TSPPlanner::computePathCost(const std::vector<Point2D> &path) {
  if (path.size() < 2) {
    return 0.0;
  }

  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i - 1].x;
    double dy = path[i].y - path[i - 1].y;
    cost += (dx != 0 && dy != 0) ? 1.414 : 1.0;
  }
  return cost;
}

void TSPPlanner::buildCostMatrix() {
  if (frontiers.empty()) {
    cost_matrix.clear();
    return;
  }

  // First, filter out unreachable frontiers from robot position
  std::vector<Point2D> reachable_frontiers;
  std::vector<size_t> frontier_indices; // Map from reachable_frontiers index to original frontiers index
  
  for (size_t i = 0; i < frontiers.size(); ++i) {
    std::vector<Point2D> test_path;
    if (aStarSearch(robot_grid_pos.x, robot_grid_pos.y, frontiers[i].x,
                    frontiers[i].y, test_path)) {
      if (test_path.size() > 1) { // Must have at least 2 points (start != goal)
        reachable_frontiers.push_back(frontiers[i]);
        frontier_indices.push_back(i);
      }
    }
  }

  if (reachable_frontiers.empty()) {
    RCLCPP_WARN(this->get_logger(), "No reachable frontiers found");
    cost_matrix.clear();
    frontiers.clear(); // Clear unreachable frontiers
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Filtered %zu reachable frontiers from %zu total",
              reachable_frontiers.size(), frontiers.size());

  // Update frontiers to only include reachable ones
  frontiers = reachable_frontiers;

  // Include robot position as the first node (index 0)
  std::vector<Point2D> all_nodes;
  all_nodes.push_back(robot_grid_pos);
  all_nodes.insert(all_nodes.end(), reachable_frontiers.begin(), reachable_frontiers.end());

  size_t n = all_nodes.size();
  cost_matrix.clear();
  cost_matrix.resize(n, std::vector<double>(n, 0.0));

  RCLCPP_INFO(this->get_logger(), "Building cost matrix for %zu nodes", n);

  // Compute pairwise distances using A*
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      std::vector<Point2D> path;
      if (aStarSearch(all_nodes[i].x, all_nodes[i].y, all_nodes[j].x,
                      all_nodes[j].y, path)) {
        double cost = computePathCost(path);
        cost_matrix[i][j] = cost;
        cost_matrix[j][i] = cost;
      } else {
        // If no path found between reachable nodes, use very high cost
        double euclidean = std::hypot(all_nodes[i].x - all_nodes[j].x,
                                      all_nodes[i].y - all_nodes[j].y);
        cost_matrix[i][j] = euclidean * 10000.0; // Very high penalty
        cost_matrix[j][i] = euclidean * 10000.0;
        RCLCPP_WARN(this->get_logger(),
                    "No path between nodes %zu and %zu, using penalty cost",
                    i, j);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Cost matrix built");
}

bool TSPPlanner::solveTSP() {
#ifdef ORTOOLS_FOUND
  if (frontiers.empty()) {
    return false;
  }

  // Include robot position
  size_t num_nodes = frontiers.size() + 1; // robot + frontiers

  if (num_nodes < 2) {
    return false;
  }

  // Create routing index manager
  operations_research::RoutingIndexManager manager(
      static_cast<int>(num_nodes), 1, 0); // num_nodes, num_vehicles, depot

  // Create routing model
  operations_research::RoutingModel routing(manager);

  // Define distance callback
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&](int64_t from_index, int64_t to_index) -> int64_t {
        int from_node = manager.IndexToNode(from_index);
        int to_node = manager.IndexToNode(to_index);
        // Convert to integer (multiply by 1000 for precision)
        return static_cast<int64_t>(cost_matrix[from_node][to_node] * 1000.0);
      });

  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Set search parameters
  operations_research::RoutingSearchParameters search_parameters =
      operations_research::DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(
      operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  search_parameters.set_local_search_metaheuristic(
      operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  search_parameters.mutable_time_limit()->set_seconds(1);

  // Solve
  const operations_research::Assignment *solution =
      routing.SolveWithParameters(search_parameters);

  if (solution == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "TSP solution not found");
    return false;
  }

  // Extract solution
  tsp_ordered_frontiers.clear();
  int64_t index = routing.Start(0);
  std::vector<int> order;
  while (routing.IsEnd(index) == false) {
    order.push_back(manager.IndexToNode(index));
    index = solution->Value(routing.NextVar(index));
  }

  // Build ordered frontier list (skip index 0 which is robot position)
  for (size_t i = 1; i < order.size(); ++i) {
    int idx = order[i];
    if (idx > 0 && idx <= static_cast<int>(frontiers.size())) {
      tsp_ordered_frontiers.push_back(frontiers[idx - 1]);
    }
  }

  RCLCPP_INFO(this->get_logger(), "TSP solved, order: %zu frontiers",
              tsp_ordered_frontiers.size());
  return true;
#else
  RCLCPP_WARN(this->get_logger(),
               "OR-Tools not found. Using nearest-neighbor fallback. "
               "Install: sudo apt-get install libortools-dev");
  // Fallback: use nearest-neighbor heuristic
  if (frontiers.empty()) {
    return false;
  }
  
  tsp_ordered_frontiers.clear();
  std::vector<bool> visited(frontiers.size(), false);
  Point2D current = robot_grid_pos;
  
  // Nearest neighbor heuristic
  for (size_t i = 0; i < frontiers.size(); ++i) {
    double min_dist = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    
    for (size_t j = 0; j < frontiers.size(); ++j) {
      if (visited[j]) continue;
      
      double dist = std::hypot(frontiers[j].x - current.x,
                               frontiers[j].y - current.y);
      if (dist < min_dist) {
        min_dist = dist;
        best_idx = j;
      }
    }
    
    visited[best_idx] = true;
    tsp_ordered_frontiers.push_back(frontiers[best_idx]);
    current = frontiers[best_idx];
  }
  
  RCLCPP_INFO(this->get_logger(), "TSP solved (fallback), order: %zu frontiers",
              tsp_ordered_frontiers.size());
  return true;
#endif
}

std::vector<Point2D> TSPPlanner::buildPathFromTSP() {
  std::vector<Point2D> full_path;
  
  if (tsp_ordered_frontiers.empty()) {
    RCLCPP_WARN(this->get_logger(), "No frontiers in TSP solution");
    return full_path;
  }

  full_path.push_back(robot_grid_pos);

  // Build path connecting all TSP-ordered frontiers
  for (const auto &frontier : tsp_ordered_frontiers) {
    Point2D current = full_path.back();
    
    // Skip if already at this frontier
    if (current.x == frontier.x && current.y == frontier.y) {
      continue;
    }
    
    std::vector<Point2D> segment;
    if (aStarSearch(current.x, current.y, frontier.x, frontier.y, segment)) {
      // Skip first point to avoid duplication
      if (!segment.empty() && segment.size() > 1) {
        full_path.insert(full_path.end(), segment.begin() + 1, segment.end());
      } else if (segment.size() == 1 && 
                 (segment[0].x != current.x || segment[0].y != current.y)) {
        full_path.push_back(segment[0]);
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to find path to frontier (%d,%d), skipping",
                  frontier.x, frontier.y);
    }
  }

  if (full_path.size() <= 1) {
    RCLCPP_WARN(this->get_logger(), 
                "Path has only %zu waypoints, may not be valid", 
                full_path.size());
  }

  return full_path;
}

void TSPPlanner::worldToGrid(const Pose &pose, int &grid_x, int &grid_y) {
  double world_x = pose.position.x;
  double world_y = pose.position.y;

  grid_x = std::floor((world_x - origin_x) / resolution);
  grid_y = std::floor((world_y - origin_y) / resolution);
}

void TSPPlanner::gridToWorld(int grid_x, int grid_y, double &world_x,
                             double &world_y) {
  world_x = origin_x + (grid_x + 0.5) * resolution;
  world_y = origin_y + (grid_y + 0.5) * resolution;
}

void TSPPlanner::publishFrontierCloud() {
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
    double world_x, world_y;
    gridToWorld(pt.x, pt.y, world_x, world_y);
    *iter_x = static_cast<float>(world_x);
    *iter_y = static_cast<float>(world_y);
    *iter_z = static_cast<float>(robot_height);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  frontier_pub->publish(cloud);
}

void TSPPlanner::publishPath(const std::vector<Point2D> &path) {
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = visible.header.frame_id;
  ros_path.header.stamp = this->now();

  double robot_height = current_pose.position.z;

  for (const auto &pt : path) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = ros_path.header;
    double world_x, world_y;
    gridToWorld(pt.x, pt.y, world_x, world_y);
    pose_stamped.pose.position.x = world_x;
    pose_stamped.pose.position.y = world_y;
    pose_stamped.pose.position.z = robot_height;
    pose_stamped.pose.orientation.w = 1.0;
    ros_path.poses.push_back(pose_stamped);
  }

  path_pub->publish(ros_path);
}

