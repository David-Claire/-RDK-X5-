import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    
    Returns:
        LaunchDescription: 完整的导航系统启动配置
    """
    
    
    # 1. 基础配置参数
    # ============================================================================
    
    # 仿真时间配置（实际机器人使用false）
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    

    # 2. 包目录和路径设置
    
    # 机器人基础包目录
    gcs_robot_dir = get_package_share_directory('turn_on_gcs_robot')
    gcs_launch_dir = os.path.join(gcs_robot_dir, 'launch')
    
    # 导航包目录
    gcs_nav_dir = get_package_share_directory('gcs_nav2')
    gcs_nav_launchr = os.path.join(gcs_nav_dir, 'launch')
    
    # ============================================================================
    # 3. 地图文件配置
    # ============================================================================
    
    # 地图文件目录和默认地图
    map_dir = os.path.join(gcs_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'gcs.yaml'))
    
    param_dir = os.path.join(gcs_nav_dir, 'param', 'gcs_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'param_mini_akm.yaml'))
    
    # ============================================================================
    # 5. 启动参数声明
    # ============================================================================
    
    # 地图文件路径参数
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='要加载的地图文件的完整路径'
    )
    
    # 参数文件路径参数
    params_arg = DeclareLaunchArgument(
        'params',
        default_value=param_file,
        description='要加载的参数文件的完整路径'
    )
    
    # ============================================================================
    # 6. 核心节点定义
    # ============================================================================
    
    # 航点循环导航节点 - 支持多个航点的循环导航
    waypoint_cycle_node = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )
    
    # ============================================================================
    # 7. 子启动文件包含
    # ============================================================================
    
    # 启动机器人基础系统（串口通信、IMU、EKF等）
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gcs_launch_dir, '/turn_on_gcs_robot.launch.py']),
    )
    
    # 启动激光雷达节点
    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gcs_launch_dir, '/gcs_lidar.launch.py']),
    )
    
    # 启动Nav2导航框架
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gcs_nav_launchr, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': param_file
        }.items(),
    )
    
    # ============================================================================
    # 8. 启动描述组装
    # ============================================================================
    
    return LaunchDescription([
        # 启动参数
        map_arg,
        params_arg,
        
        # 航点循环导航节点
        waypoint_cycle_node,
        
        # 机器人基础系统
        robot_bringup,
        
        # 激光雷达系统
        lidar_bringup,
        
        # Nav2导航框架
        nav2_bringup,
    ])