import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def generate_launch_description():
    """
    
    Returns:
        LaunchDescription: 完整的启动配置
    """
    
    # ============================================================================
    # 1. 目录和文件路径设置
    # ============================================================================
    
    # 获取包目录和启动文件路径
    bringup_dir = get_package_share_directory('turn_on_gcs_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # 不同组件的配置文件
    ekf_config = Path(get_package_share_directory('turn_on_gcs_robot'), 'config', 'ekf.yaml')
    ekf_carto_config = Path(get_package_share_directory('turn_on_gcs_robot'), 'config', 'ekf_carto.yaml')
    imu_config = Path(get_package_share_directory('turn_on_gcs_robot'), 'config', 'imu.yaml')

    # ============================================================================
    # 2. 启动参数配置
    # ============================================================================
    
    # 启动参数：启用/禁用建图SLAM功能
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument(
        'carto_slam',
        default_value='false',
        description='启用Cartographer SLAM建图功能'
    )

    # ============================================================================
    # 3. 核心机器人系统组件
    # ============================================================================
    
    # 基础机器人控制 - 处理与机器人硬件的串口通信
    gcs_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
        launch_arguments={}.items(),
    )
    
    # 扩展卡尔曼滤波器用于传感器融合（里程计、IMU等）
    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gcs_ekf.launch.py')),
        launch_arguments={'carto_slam': carto_slam}.items(),
    )

    # ============================================================================
    # 4. 坐标变换发布器（TF2）
    # ============================================================================
 
    base_to_link = launch_ros.actions.Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    
    # 从base_footprint到gyro_link（IMU坐标系）的静态变换
    base_to_gyro = launch_ros.actions.Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_gyro',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'gyro_link'],
    )

    # ============================================================================
    # 5. 传感器处理节点
    # ============================================================================
    
    # IMU滤波器，使用Madgwick算法进行姿态估计
    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[imu_config]
    )
    
    # 机器人关节状态发布器，用于运动学计算
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher', 
        executable='joint_state_publisher', 
        name='joint_state_publisher',
    )

    minibot_type = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description_minibot.launch.py')),
        launch_arguments={'mini_mec': 'true'}.items(),
    )


    flagship_type = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
        launch_arguments={'senior_akm': 'true'}.items(),
    )

    # ============================================================================
    # 7. 启动描述组装
    # ============================================================================
    
    # 创建启动描述并添加所有组件
    ld = LaunchDescription()

    # 添加启动参数
    ld.add_action(carto_slam_dec)
    
    ld.add_action(minibot_type)        # 启用小型机器人
    # ld.add_action(flagship_type)     # 启用旗舰机器人（已注释）
    
    # 添加核心机器人系统
    ld.add_action(gcs_robot)      # 基础机器人控制
    ld.add_action(robot_ekf)           # 扩展卡尔曼滤波器
    
    # 添加坐标变换发布器
    ld.add_action(base_to_link)        # 基础坐标系变换
    ld.add_action(base_to_gyro)        # IMU坐标系变换
    
    # 添加传感器处理和状态发布器
    ld.add_action(joint_state_publisher_node)  # 关节状态发布
    ld.add_action(imu_filter_node)             # IMU滤波处理

    return ld