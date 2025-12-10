from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime

def generate_launch_description():
    simulator_share = get_package_share_directory('simulator')
    essential_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, 'launch', 'indoor1.center.sim_essential.launch.py')
        )
    )
    rviz_config_file = os.path.join(
        simulator_share,
        'rviz',
        'mp3d_find_nearest.rviz'
    )

    return LaunchDescription(
        [
            essential_launch,
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', rviz_config_file]
            ),
            Node(
                package='dev_hpp',
                executable='no_predicted_local_planner_node',
                name='local_planner',
                output='screen',
                parameters=[{
                    'global_eps': 4.0,
                    'local_eps': 2.0,
                    'local_min_points': 2,
                    'global_min_points': 5,
                    'max_range': 20.0,
                    'replan_threshold': 1.0,
                    'time_threshold': 1.5,
                    'frontier_detection_range': -1.0,
                    'global_planning_failure_threshold': 1,
                    'local_cluster_size': 12,
                    'global_cluster_size': 30,
                    'direction_change_penalty': 0.0,
                    'predicted_confidence_multiplier': 0.6
                }]
            ),
            Node(
                package='local_planner',
                executable='path_selector_node',
                name='path_selector',
                output='screen',
                parameters=[{
                    'robot_name':'av1',
                    'height': 1.25,
                    'waypoint_tolerance': 0.2,
                    'direction_change_penalty': 0.0
                }]
            ),
            Node(
                package='dev_hpp',
                executable='global_planner_node',
                name='global_planner',
                output='screen',
                parameters=[{
                    'global_range': 6.0
                }]
            ),
            Node(
                package="exp_statistics",
                executable="exp_statistics_node",
                name="exp_statistics",
                output="screen",
                parameters=[
                    {
                        "stat_file_path": os.path.join(
                        os.path.dirname(os.path.dirname((os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))),
                            "src",
                            "simulator",
                            "statistics",
                            "indoor1",
                            f"hpp_stats_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                        )
                    }
                ],
            )
        ]
    )
