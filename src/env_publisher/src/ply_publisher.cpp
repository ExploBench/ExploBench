#include "../include/ply_publisher.h"
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

PLYPublisher::PLYPublisher() : Node("pointcloud_publisher") {
  // pointcloud file path
  this->declare_parameter<std::string>("pointcloud_file", "output.ply");
  this->declare_parameter<double>("local_radius", 18.0);
  this->declare_parameter<double>("scan_resolution", 10.0);
  this->declare_parameter<double>("sample_step", 2.0);
  this->declare_parameter<double>("x_offset", 0.0);
  this->declare_parameter<double>("y_offset", 0.0);
  this->declare_parameter<double>("scaler", 10.0);
  pointcloud_file = this->get_parameter("pointcloud_file").as_string();
  local_radius = this->get_parameter("local_radius").as_double();
  scan_resolution = this->get_parameter("scan_resolution").as_double();
  sample_step = this->get_parameter("sample_step").as_double();
  x_offset = this->get_parameter("x_offset").as_double();
  y_offset = this->get_parameter("y_offset").as_double();
  scaler = this->get_parameter("scaler").as_double();

  // get absolute path of pcd file
  std::string package = "env_publisher";
  std::string absolute_path;
  try {
    absolute_path = ament_index_cpp::get_package_share_directory(package) +
                    "/" + pointcloud_file;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Package %s not found: %s",
                 package.c_str(), e.what());
    rclcpp::shutdown();
    return;
  }

  // load cloud
  pcl::PLYReader reader;
  if (reader.read(absolute_path, cloud) == -1) {
    PCL_ERROR("Couldn't read PLY file %s \n", pointcloud_file.c_str());
    return;
  }

  // load cloud from pcd file
  // if (pcl::io::loadPCDFile<PointT>(absolute_path, cloud) == -1) {
  //   PCL_ERROR("Couldn't read file %s \n", pointcloud_file.c_str());
  //   return;
  // }

  for (auto &p : cloud.points) {
    p.x *= scaler; // scale by 10
    p.x += x_offset;
    p.y *= scaler;
    p.y += y_offset;
    p.z *= scaler;
  }

  // initialize vehicles vector
  vehicles = {"av1"};
  for (auto &name : vehicles) {
    std::string local_pointcloud_topic = "/" + name + "/local_pointcloud";
    std::string odom_topic = "/" + name + "/odom";
    local_pointcloud_pubs[name] =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            local_pointcloud_topic, 1);
    odom_subs[name] = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this, name](nav_msgs::msg::Odometry::SharedPtr msg) {
          odomCallback(msg, name);
        });
  }

  // pointcloud publisher
  pointcloud_publisher =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_env", 10);

  // create timer at 1 Hz
  timer = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&PLYPublisher::timerCallback, this));

  // initialize kdtree
  kdtree.setInputCloud(cloud.makeShared());
}

void PLYPublisher::timerCallback() {
  // convert pointcloud
  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(cloud, ros_pointcloud);
  ros_pointcloud.header.frame_id = "map";
  ros_pointcloud.header.stamp = this->get_clock()->now();
  pointcloud_publisher->publish(ros_pointcloud);

  // publish local pointcloud
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto &name : vehicles) {
    auto it = current_positions.find(name);
    if (it != current_positions.end()) {
      Eigen::Vector3f position = it->second;

      pcl::PointCloud<PointT>::Ptr local_cloud(new pcl::PointCloud<PointT>());

      for (float angle = 0.0f; angle < 360.0f;
           angle += static_cast<float>(scan_resolution)) {
        float yaw_rad = angle * static_cast<float>(M_PI) / 180.0f;

        for (float v_angle = -90.0f;
             v_angle <= static_cast<float>(vertical_angle); v_angle += 10.0f) {
          float pitch_rad = v_angle * static_cast<float>(M_PI) / 180.0f;
          Eigen::Vector3f ray_direction(std::cos(yaw_rad) * std::cos(pitch_rad),
                                        std::sin(yaw_rad) * std::cos(pitch_rad),
                                        std::sin(pitch_rad));
          ray_direction.normalize();

          for (float t = sample_step; t <= local_radius; t += 2.0f) {
            Eigen::Vector3f sample_point = position + t * ray_direction;
            PointT search_point;
            search_point.x = sample_point[0];
            search_point.y = sample_point[1];
            search_point.z = sample_point[2];

            std::vector<int> point_idx;
            std::vector<float> point_dist;
            int num_neighbors = kdtree.radiusSearch(search_point, 12.0f,
                                                    point_idx, point_dist, 250);

            if (num_neighbors > 0) {
              PointT collision_point;
              for (int i = 0; i < num_neighbors; ++i) {
                collision_point = cloud.points[point_idx[i]];
                local_cloud->points.push_back(collision_point);
              }

              // optional noise points
              int num_noise_points = 0;
              float noise_range = 0.1f;
              for (int i = 0; i < num_noise_points; ++i) {
                PointT noise_point;
                noise_point.x =
                    collision_point.x +
                    ((rand() / static_cast<float>(RAND_MAX)) * 2 - 1) *
                        noise_range;
                noise_point.y =
                    collision_point.y +
                    ((rand() / static_cast<float>(RAND_MAX)) * 2 - 1) *
                        noise_range;
                noise_point.z =
                    collision_point.z +
                    ((rand() / static_cast<float>(RAND_MAX)) * 2 - 1) *
                        noise_range;
                local_cloud->points.push_back(noise_point);
              }
              // stop along this ray when collision found
              break;
            }
          }
        }
      }

      if (!local_cloud->points.empty()) {
        sensor_msgs::msg::PointCloud2 local_msg;
        pcl::toROSMsg(*local_cloud, local_msg);
        local_msg.header.stamp = this->now();
        local_msg.header.frame_id = "map";
        local_pointcloud_pubs[name]->publish(local_msg);
      }
    }
  }
}

void PLYPublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                                const std::string &name) {
  Eigen::Vector3f position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  std::lock_guard<std::mutex> lock(mutex);
  current_positions[name] = position;
}