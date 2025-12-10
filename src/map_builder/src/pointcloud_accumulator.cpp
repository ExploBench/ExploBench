#include <memory>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PclPoint = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PclPoint>;
using PointCloudPtr = PclCloud::Ptr;

class PointCloudAccumulator : public rclcpp::Node {
public:
  PointCloudAccumulator() : Node("pointcloud_accumulator"), voxel_size_(0.1) {
    // Initialize accumulated cloud
    accumulated_cloud_ = std::make_shared<PclCloud>();

    // Create subscription to /av1/local_pointcloud
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/av1/local_pointcloud", 10,
        std::bind(&PointCloudAccumulator::cloudCallback, this,
                  std::placeholders::_1));

    // Create publisher for accumulated cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/accumulated_pointcloud", 10);

    // Create timer to publish accumulated cloud periodically
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PointCloudAccumulator::publishAccumulatedCloud, this));

    RCLCPP_INFO(this->get_logger(), "PointCloud Accumulator Node started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /av1/local_pointcloud");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /accumulated_pointcloud");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Convert ROS2 message to PCL cloud
    PointCloudPtr incoming_cloud = std::make_shared<PclCloud>();
    pcl::fromROSMsg(*msg, *incoming_cloud);

    // Add incoming points to accumulated cloud
    *accumulated_cloud_ += *incoming_cloud;

    // Apply downsampling when cloud gets large
    if (accumulated_cloud_->size() > 100000) {  // Downsample when > 100k points
      downsampleCloud();
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "Received cloud with %zu points. Total accumulated: %zu points",
        incoming_cloud->size(), accumulated_cloud_->size());
  }

  void downsampleCloud() {
    if (accumulated_cloud_->empty()) {
      return;
    }

    // Create voxel grid filter
    pcl::VoxelGrid<PclPoint> voxel_filter;
    voxel_filter.setInputCloud(accumulated_cloud_);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

    // Apply filter and store result back
    PointCloudPtr filtered_cloud = std::make_shared<PclCloud>();
    voxel_filter.filter(*filtered_cloud);

    accumulated_cloud_ = filtered_cloud;

    RCLCPP_INFO(this->get_logger(),
                "Downsampled cloud to %zu points (voxel size: %.2fm)",
                accumulated_cloud_->size(), voxel_size_);
  }

  void publishAccumulatedCloud() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (accumulated_cloud_->empty()) {
      return;
    }

    // Convert PCL cloud to ROS2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*accumulated_cloud_, output_msg);
    output_msg.header.stamp = this->get_clock()->now();
    output_msg.header.frame_id = "map"; // You may want to adjust this

    // Publish accumulated cloud
    publisher_->publish(output_msg);

    RCLCPP_INFO(this->get_logger(),
                "Published accumulated cloud with %zu points",
                accumulated_cloud_->size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  PointCloudPtr accumulated_cloud_;
  std::mutex mutex_;

  // Downsampling parameters
  double voxel_size_;  // Size of voxel grid for downsampling (in meters)
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudAccumulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}