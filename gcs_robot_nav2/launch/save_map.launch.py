#ros2 run nav2_map_server map_saver_cli -f ~/map
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions


def generate_launch_description():

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', '/home/gcs/gcs_ros2/install/gcs_nav2/share/gcs_nav2/map/gcs'],
        
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    map_backup = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_backup',
        output='screen',
        arguments=['-f', '/home/gcs/gcs_ros2/src/gcs_robot_nav2/map/gcs'],
        
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)
    ld.add_action(map_backup)
    return ld
 
