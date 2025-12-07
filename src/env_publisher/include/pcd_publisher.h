#ifndef PCD_PUBLISHER_H
#define PCD_PUBLISHER_H

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

using PointT = pcl::PointXYZRGB;
using PointCloud2PublisherPtr =
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
using OdometrySubscriberPtr =
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher();

private:
  // publisher of global pointcloud
  PointCloud2PublisherPtr pointcloud_publisher;

  // publisher of local pointcloud
  std::unordered_map<std::string, PointCloud2PublisherPtr>
      local_pointcloud_pubs;
  std::vector<std::string> vehicles;

  // subscriber of odom of each vehicle
  std::unordered_map<std::string, OdometrySubscriberPtr> odom_subs;
  std::unordered_map<std::string, Eigen::Vector3f> current_positions;
  // odom callback
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                    const std::string &name);

  // timer
  rclcpp::TimerBase::SharedPtr timer;
  // timer callback
  void timerCallback();

  // pointcloud file
  std::string pointcloud_file;
  // global pointcloud
  pcl::PointCloud<PointT> cloud;
  pcl::KdTreeFLANN<PointT> kdtree;

  // thread lock
  std::mutex mutex;

  // lidar parameters
  double local_radius;
  double scan_resolution;
  double sample_step;
  // maximum angle to look up
  double vertical_angle = 90.0;
  double vertical_resolution;
  double scaler, x_offset, y_offset;
};

#endif