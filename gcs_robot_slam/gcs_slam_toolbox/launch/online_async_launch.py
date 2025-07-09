from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取机器人功能包的路径
    bringup_dir = get_package_share_directory('turn_on_gcs_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 加载机器人主控制器的launch文件
    gcs_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'turn_on_gcs_robot.launch.py')
        )
    )

    # 加载LiDAR传感器的launch文件
    gcs_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gcs_lidar.launch.py')
        )
    )

    # 创建slam_toolbox节点（在线异步SLAM）
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',  # 将日志输出到屏幕
        parameters=[
            # 加载参数配置文件路径
            os.path.join(
                get_package_share_directory("gcs_slam_toolbox"),
                'config',
                'mapper_params_online_async.yaml'
            )
        ],
        remappings=[('odom', 'odom_combined')]  # 话题重映射
    )

    # 返回完整的Launch描述，包含以上3部分
    return LaunchDescription([
        gcs_robot,
        gcs_lidar,
        slam_node
    ])
