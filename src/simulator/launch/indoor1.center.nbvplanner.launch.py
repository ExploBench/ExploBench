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
        'show.rviz'
    )

    return LaunchDescription(
        [
            essential_launch,
            Node(
                package="nbvplanner",
                executable="nbvplanner_node",
                name="nbvplanner",
                output="screen"
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', rviz_config_file]
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
                            f"nbvplanner_stats_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                        )
                    }
                ],
            )
        ]
    )
