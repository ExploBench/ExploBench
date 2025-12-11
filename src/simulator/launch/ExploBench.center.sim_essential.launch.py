from launch import LaunchDescription
from launch_ros.actions import Node
import os
import math


def generate_launch_description():
    height = 1.5
    resolution = 0.5

    return LaunchDescription(
        [
            Node(
                package="env_publisher",
                executable="pcd_publisher_node",
                name="pcd_publisher",
                output="screen",
                parameters=[
                    {
                        "pointcloud_file": "env/ExploBench.pcd",
                        "local_radius": 20.0,
                        "scan_resolution": 10.0,
                        "sample_step": 2.0,
                        "x_offset": -55.0,
                        "y_offset": -30.0,
                        "scaler": 1.1,
                    }
                ],
            ),
            Node(
                package="map_builder",
                executable="map_builder_node",
                name="map_builder",
                output="screen",
                parameters=[
                    {
                        "resolution": resolution,
                        "size_x": 221.0,
                        "size_y": 101.0,
                        "height": height,
                        "num_rays": 120.0,
                        "max_range": 12.0,
                    }
                ],
            ),
            Node(
                package="odom_manager",
                executable="odom_manager_node",
                name="odom_manager",
                output="screen",
                parameters=[
                    {
                        "av1/x_init": -30.0,  # 60
                        "av1/y_init": -30.0,  # 20
                        "av1/z_init": height,
                        "av1/yaw": 5 * math.pi / 4,
                        "resolution": resolution,
                    }
                ],
            ),
            Node(
                package="local_planner",
                executable="path_tracker_node",
                name="path_tracker",
                output="screen",
                parameters=[
                    {
                        "robot_name": "av1",
                        "kp_linear": 2.5,
                        "kp_angular": 3.0,
                        "kp_angular_small": 1.0,
                        "tolerance": 0.5,
                        "max_linear_velocity": 3.0,
                        "max_angular_velocity": 3.0,
                    }
                ],
            ),
            Node(
                package="map_builder",
                executable="pointcloud_accumulator_node",
                name="pointcloud_accumulator",
                output="screen"
            )
        ]
    )
