from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 turn_on_gcs_robot 包的共享路径
    bringup_dir = get_package_share_directory('turn_on_gcs_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 加载机器人本体的启动文件
    gcs_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'turn_on_gcs_robot.launch.py')
        )
    )

    # 加载激光雷达的启动文件
    gcs_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gcs_lidar.launch.py')
        )
    )

    # 配置同步 SLAM 工具箱节点
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # 注意这是同步模式
        name='slam_toolbox',
        output='screen',  # 日志输出到终端
        parameters=[
            os.path.join(
                get_package_share_directory("gcs_slam_toolbox"),
                'config',
                'mapper_params_online_sync.yaml'  # 加载同步SLAM参数
            )
        ],
        remappings=[('odom', 'odom_combined')]  # 将里程计话题重映射
    )

    # 返回整个Launch描述，包括机器人、雷达和SLAM节点
    return LaunchDescription([
        gcs_robot,
        gcs_lidar,
        slam_node
    ])
