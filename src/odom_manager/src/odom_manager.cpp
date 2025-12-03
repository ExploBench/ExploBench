#include "../include/odom_manager.h"

#include "../include/vehicle.h"

OdomManager::OdomManager() : Node("odom_manager"), rate(100.0) {
  // initial position
  double av1_x, av1_y, av1_z, av1_yaw;
  this->declare_parameter("av1/x_init", 0.0);
  this->declare_parameter("av1/y_init", 0.0);
  this->declare_parameter("av1/z_init", 0.3);
  this->declare_parameter("av1/yaw", 0.0);
  this->declare_parameter("resolution", 0.0);
  av1_x = this->get_parameter("av1/x_init").as_double();
  av1_y = this->get_parameter("av1/y_init").as_double();
  av1_z = this->get_parameter("av1/z_init").as_double();
  av1_yaw = this->get_parameter("av1/yaw").as_double();
  resolution = this->get_parameter("resolution").as_double();

  // initialize vehicle definition
  vehicle_definitions = {
      {"av1", InitialStatus{av1_x * resolution, av1_y * resolution,
                            av1_z * resolution, av1_yaw, VehicleType::AV, true,
                            "package://odom_manager/model/av.dae"}}};
}

void OdomManager::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg,
                              const std::string &vehicle_name) {
  auto it = vehicles.find(vehicle_name);
  if (it != vehicles.end()) {
    it->second->vx = msg->linear.x;
    it->second->vy = msg->linear.y;
    it->second->vz = msg->linear.z;
    it->second->angular_velocity = msg->angular.z;
  }
}

void OdomManager::run() {
  tf2_ros::TransformBroadcaster broadcaster(this);
  auto prev_time = this->now();

  while (rclcpp::ok()) {
    auto current_time = this->now();
    double dt = (current_time - prev_time).seconds();
    prev_time = current_time;
    current_time = this->now();
    if (dt <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Detected ROS time jump backwards dt=%f (skipping update)",
                  dt);
      continue;
    }

    for (auto &pair : vehicles) {
      pair.second->updatePose(dt);
      pair.second->publishData(current_time, broadcaster);
    }

    rclcpp::spin_some(this->shared_from_this());
    rate.sleep();
  }
}

void OdomManager::initVehicles() {
  for (auto &[name, status] : vehicle_definitions) {
    if (status.type == VehicleType::AV) {
      vehicles.emplace(
          name, std::make_shared<AV>(name, resolution, this->shared_from_this(),
                                     status.x_init, status.y_init,
                                     status.z_init, status.theta_init,
                                     status.use_model, status.mesh_resource));
    }
    vehicles[name]->cmd_vel_sub =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/" + name + "/cmd_vel", 10,
            [this, name](const geometry_msgs::msg::Twist::SharedPtr msg) {
              this->cmdCallback(msg, name);
            });
  }
}