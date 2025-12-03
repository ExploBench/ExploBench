#include "../include/nbvplanner.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NBVPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}