#include "../include/odom_manager.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto manager = std::make_shared<OdomManager>();
  manager->initVehicles();
  manager->run();
  rclcpp::shutdown();
  return 0;
}
