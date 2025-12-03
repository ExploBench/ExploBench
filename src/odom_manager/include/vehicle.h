#ifndef VEHICLE_H
#define VEHICLE_H

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <string>
#include <tf2_ros/transform_broadcaster.h>

using OdomPublisherPtr = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;
using VisPublisherPtr =
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;
using VelSubscriberPtr =
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;

enum class VehicleType { GV, AV };

struct InitialStatus {
  double x_init;
  double y_init;
  double z_init;
  double theta_init;
  VehicleType type;
  bool use_model;
  std::string mesh_resource;
};

struct Vehicle {
  // basic info of a robot
  std::string name;
  double resolution;
  // location and velocity
  double x, y, z, theta;
  double vx, vy, vz, angular_velocity;

  // subscriber and publisher
  OdomPublisherPtr odom_pub;
  VisPublisherPtr marker_pub;
  VisPublisherPtr trajectory_pub;
  VelSubscriberPtr cmd_vel_sub;

  // Marker for visualization
  visualization_msgs::msg::Marker trajectory_marker;

  // model
  bool use_model;
  std::string mesh_resource;

  // constructor and destructor
  Vehicle(const std::string &vehicle_name, double resolution,
          std::shared_ptr<rclcpp::Node> node, double x_init, double y_init,
          double z_init, double theta_init, bool use_model_flag = false,
          const std::string &mesh_res = "");
  virtual ~Vehicle();

  virtual void updatePose(double dt) = 0;
  virtual void publishData(const rclcpp::Time &current_time,
                           tf2_ros::TransformBroadcaster &broadcaster) = 0;
};

struct AV : public Vehicle {
  AV(const std::string &vehicle_name, double resolution,
     std::shared_ptr<rclcpp::Node> node, double x_init, double y_init,
     double z_init, double theta_init, bool use_model_flag = false,
     const std::string &mesh_res = "");

  void updatePose(double dt) override;
  void publishData(const rclcpp::Time &current_time,
                   tf2_ros::TransformBroadcaster &broadcaster) override;
};

#endif