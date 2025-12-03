#include "../include/vehicle.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

Vehicle::Vehicle(const std::string &vehicle_name, double resolution,
                 std::shared_ptr<rclcpp::Node> node, double x_init,
                 double y_init, double z_init, double theta_init,
                 bool use_model_flag, const std::string &mesh_res)
    : name(vehicle_name), resolution(resolution), x(x_init), y(y_init),
      z(z_init), theta(theta_init), vx(0.0), vy(0.0), vz(0.0),
      angular_velocity(0.0), use_model(use_model_flag),
      mesh_resource(mesh_res) {
  odom_pub =
      node->create_publisher<nav_msgs::msg::Odometry>("/" + name + "/odom", 10);
  marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      "/" + name + "/vehicle_marker", 10);
  trajectory_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      "/" + name + "/vehicle_trajectory", 10);

  trajectory_marker.header.frame_id = "map";
  trajectory_marker.header.stamp = node->get_clock()->now();
  trajectory_marker.ns = name + "_trajectory";
  trajectory_marker.id = 0;
  trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0; // no rotation

  trajectory_marker.color.r = 0.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;
  trajectory_marker.color.a = 1.0;

  trajectory_marker.scale.x = 0.5 * resolution;
  trajectory_marker.points.clear();
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  trajectory_marker.points.push_back(p);
}

Vehicle::~Vehicle() {}

AV::AV(const std::string &vehicle_name, double resolution,
       std::shared_ptr<rclcpp::Node> node, double x_init, double y_init,
       double z_init, double theta_init, bool use_model_flag,
       const std::string &mesh_res)
    : Vehicle::Vehicle(vehicle_name, resolution, node, x_init, y_init, z_init,
                       theta_init, use_model_flag, mesh_res) {}

void AV::updatePose(double dt) {
  double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
  double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
  double delta_z = 0.0;

  // skip update if large jump detected
  const double max_delta = 1.0; // meters
  if (std::abs(delta_x) > max_delta || std::abs(delta_y) > max_delta) {
    return;
  }

  // update pose
  x += delta_x;
  y += delta_y;
  z += delta_z;
  theta += angular_velocity * dt;
  theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;

  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  trajectory_marker.points.push_back(p);
}

void AV::publishData(const rclcpp::Time &current_time,
                     tf2_ros::TransformBroadcaster &broadcaster) {
  // odom
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "map";
  odom.child_frame_id = name + "_base_link";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;

  tf2::Quaternion q_tf2;
  q_tf2.setRPY(0, 0, theta);
  odom.pose.pose.orientation = tf2::toMsg(q_tf2);

  std::fill(std::begin(odom.pose.covariance), std::end(odom.pose.covariance),
            0.0);
  std::fill(std::begin(odom.twist.covariance), std::end(odom.twist.covariance),
            0.0);

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = angular_velocity;

  odom_pub->publish(odom);

  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = name + "_base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;
  odom_trans.transform.rotation = odom.pose.pose.orientation;

  broadcaster.sendTransform(odom_trans);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = name + "_marker";
  marker.id = 0;

  if (use_model && !mesh_resource.empty()) {
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh_resource;
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
  } else {
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }

  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation = odom.pose.pose.orientation;

  marker_pub->publish(marker);
  trajectory_pub->publish(trajectory_marker);
}