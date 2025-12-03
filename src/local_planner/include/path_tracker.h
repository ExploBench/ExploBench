#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <string>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using OdomSubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;
using PathSubscriberPtr = rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr;
using StringSubscriberPtr =
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
using TwistPublisherPtr =
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using MarkerPublisherPtr =
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;

class PathTracker : public rclcpp::Node {
public:
  PathTracker();

private:
  // manage robot
  std::string robot;

  // subscribe to odom
  geometry_msgs::msg::Pose current_pose;
  OdomSubscriberPtr odom_subscriber;
  bool odom_received;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // subscribe to local path
  PathSubscriberPtr local_path_subscriber;
  nav_msgs::msg::Path path, last_path;
  size_t current_index = 0;
  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // subscribe to adjust signal
  StringSubscriberPtr adjust_subscriber;
  std::string status = "IN";
  void adjustCallback(const std_msgs::msg::String::SharedPtr msg);

  // publish twist (cmd_vel)
  TwistPublisherPtr cmd_vel_publisher;
  void send_cmd_vel(double v, double w);

  // trakcing parameter, we apply a P controller
  double kp_linear;
  double kp_angular;
  double kp_angular_small;
  double tolerance;
  // dynamic limit
  double max_linear_velocity, max_angular_velocity;

  // publish marker for goal point
  MarkerPublisherPtr target_point_publisher, all_waypoints_publisher,
      tangent_line_publisher;
  void publishTargetPointMarker(double x, double y, double z);
  void publishAllWaypointsMarker();
  void publishTangentLine(double idx_x, double idx_y, double idx_z,
                          double next_x, double next_y, double next_z);
};

#endif