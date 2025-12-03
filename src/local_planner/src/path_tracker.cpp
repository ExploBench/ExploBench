#include "../include/path_tracker.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

PathTracker::PathTracker() : Node("path_tracker_node"), odom_received(false) {
  // initialize name
  this->declare_parameter<std::string>("robot_name", "av1");
  robot = this->get_parameter("robot_name").as_string();

  // subscribe to odom
  std::string odom_topic = "/" + robot + "/odom";
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&PathTracker::odomCallback, this, std::placeholders::_1));

  // subscribe to local path
  std::string path_topic = "/" + robot + "/planned_path";
  local_path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      std::bind(&PathTracker::localPathCallback, this, std::placeholders::_1));

  // subscribe to adjust signal
  std::string adjust_topic = "/" + robot + "/adjust";
  adjust_subscriber = this->create_subscription<std_msgs::msg::String>(
      adjust_topic, 10,
      std::bind(&PathTracker::adjustCallback, this, std::placeholders::_1));

  // publish cmd_vel
  std::string cmd_vel_topic = "/" + robot + "/cmd_vel";
  cmd_vel_publisher =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  // publish target point marker
  std::string target_point_topic = "/" + robot + "/target_point";
  target_point_publisher =
      this->create_publisher<visualization_msgs::msg::Marker>(
          target_point_topic, 10);

  // publish tangent line marker
  // std::string tangent_line_topic = "/" + robot + "/tangent_line";
  // tangent_line_publisher =
  //     this->create_publisher<visualization_msgs::msg::Marker>(
  //         tangent_line_topic, 10);

  std::string all_waypoints_topic = "/" + robot + "/all_waypoints";
  all_waypoints_publisher =
      this->create_publisher<visualization_msgs::msg::Marker>(
          all_waypoints_topic, 10);

  // initialize parameters
  this->declare_parameter<double>("kp_linear", 1.0);
  this->declare_parameter<double>("kp_angular", 1.0);
  this->declare_parameter<double>("kp_angular_small", 0.5);
  this->declare_parameter<double>("tolerance", 0.05);
  this->declare_parameter<double>("max_linear_velocity", 0.5);
  this->declare_parameter<double>("max_angular_velocity", 1.0);
  this->get_parameter("kp_linear", kp_linear);
  this->get_parameter("kp_angular", kp_angular);
  this->get_parameter("kp_angular_small", kp_angular_small);
  this->get_parameter("tolerance", tolerance);
  this->get_parameter("max_linear_velocity", max_linear_velocity);
  this->get_parameter("max_angular_velocity", max_angular_velocity);
}

void PathTracker::adjustCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (status == msg->data) {
    return;
  }
  status = msg->data;
}

void PathTracker::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  last_path = path;
  path.poses.clear();
  if (msg->poses.empty() && status == "IN") {
    send_cmd_vel(0, 0);
    return;
  }
  path = *msg;

  // only reset current_index if
  // 1. it's out of bounds or
  if (current_index >= path.poses.size()) {
    current_index = 0;
    RCLCPP_INFO(this->get_logger(),
                "Current index out of bounds. Resetting to 0.");
  }
  // 2. if this is a completely new path
  if (!last_path.poses.empty() &&
      path.poses.back().pose.position != last_path.poses.back().pose.position) {
    current_index = 1;
    RCLCPP_INFO(this->get_logger(),
                "Path updated. Resetting current index to 0.");
  }
}

void PathTracker::send_cmd_vel(double v, double w) {
  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = v;
  target_velocity.angular.z = w;
  cmd_vel_publisher->publish(target_velocity);
}

void PathTracker::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose = msg->pose.pose;
  odom_received = true;

  // if (status != "IN") {
  //   if (status == "LEFT") {
  //     send_cmd_vel(0, 1.0);
  //   } else if (status == "RIGHT") {
  //     send_cmd_vel(0, -1.0);
  //   }
  //   return;
  // }

  if (path.poses.empty()) {
    send_cmd_vel(0, 0);
    return;
  }

  // safety check: ensure we have valid path data
  if (path.poses.empty() || current_index >= path.poses.size()) {
    current_index = 0;
    RCLCPP_INFO(
        this->get_logger(),
        "No valid path or current index out of bounds. Stopping robot.");
    send_cmd_vel(0, 0);
    return;
  }

  size_t idx = current_index;

  // calculate dx, dy
  double dx = path.poses[idx].pose.position.x - current_pose.position.x;
  double dy = path.poses[idx].pose.position.y - current_pose.position.y;
  double distance = sqrt(dx * dx + dy * dy);

  // only skip waypoint if we've reached it closely enough
  if (distance < tolerance && idx < path.poses.size() - 1) {
    current_index++;
    // next callback will handle the new target
    return;
  } else if (distance < tolerance && idx == path.poses.size() - 1) {
    send_cmd_vel(0, 0);
    return;
  }

  // compute current robot orientation
  tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y,
                    current_pose.orientation.z, current_pose.orientation.w);
  double roll, pitch, current_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

  // target angle
  double target_angle = std::atan2(dy, dx);

  // error angle normalization to [-pi, pi]
  double error_angle = target_angle - current_yaw;
  while (error_angle > M_PI)
    error_angle -= 2 * M_PI;
  while (error_angle < -M_PI)
    error_angle += 2 * M_PI;

  // velocity control with better handling of large angle errors
  const double LARGE_ANGLE_THRESHOLD = M_PI / 6; // 30 degree

  double linear_vel = kp_linear * distance;
  double angular_vel = (fabs(error_angle) >= LARGE_ANGLE_THRESHOLD)
                           ? kp_angular * error_angle
                           : kp_angular_small * error_angle;

  // only stop forward motion for very large angle errors
  if (fabs(error_angle) >= LARGE_ANGLE_THRESHOLD) {
    linear_vel *= 0.05;
  }
  if (fabs(error_angle) >= M_PI / 2) {
    linear_vel = 0.0; // stop if the angle error is greater than 90 degrees
  }

  if (linear_vel > max_linear_velocity)
    linear_vel = max_linear_velocity;
  if (angular_vel > max_angular_velocity)
    angular_vel = max_angular_velocity;
  if (angular_vel < -max_angular_velocity)
    angular_vel = -max_angular_velocity;

  send_cmd_vel(linear_vel, angular_vel);
  publishTargetPointMarker(path.poses[idx].pose.position.x,
                           path.poses[idx].pose.position.y,
                           path.poses[idx].pose.position.z);
  publishAllWaypointsMarker();
}

void PathTracker::publishTargetPointMarker(double x, double y, double z) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = robot + "/target_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  target_point_publisher->publish(marker);
}

void PathTracker::publishAllWaypointsMarker() {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = robot + "/all_waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (const auto &pose_stamped : path.poses) {
    geometry_msgs::msg::Point p;
    p.x = pose_stamped.pose.position.x;
    p.y = pose_stamped.pose.position.y;
    p.z = pose_stamped.pose.position.z;
    marker.points.push_back(p);
  }

  all_waypoints_publisher->publish(marker);
}

void PathTracker::publishTangentLine(double idx_x, double idx_y, double idx_z,
                                     double next_x, double next_y,
                                     double next_z) {
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = this->now();
  line_marker.ns = robot + "/tangent_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point p_start;
  p_start.x = idx_x;
  p_start.y = idx_y;
  p_start.z = idx_z;

  geometry_msgs::msg::Point p_end;
  p_end.x = next_x;
  p_end.y = next_y;
  p_end.z = next_z;

  line_marker.points.clear();
  line_marker.points.push_back(p_start);
  line_marker.points.push_back(p_end);

  line_marker.scale.x = 0.02;
  line_marker.color.a = 1.0;
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 1.0;

  tangent_line_publisher->publish(line_marker);
}
