#ifndef ODOM_MANAGER_H
#define ODOM_MANAGER_H

#include "vehicle.h"

class OdomManager : public rclcpp::Node {
public:
  OdomManager();
  void run();
  void initVehicles();

private:
  rclcpp::Rate rate;

  std::map<std::string, std::shared_ptr<Vehicle>> vehicles;
  std::map<std::string, InitialStatus> vehicle_definitions;

  double resolution;

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg,
                   const std::string &vehicle_name);
};

#endif