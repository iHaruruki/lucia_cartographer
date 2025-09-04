#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_cartographer')

    default_config_dir = os.path.join(pkg_share, 'config')
    default_config_basename = 'unilidar_3d.lua'
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    points2_topic = LaunchConfiguration('points2')
    imu_topic = LaunchConfiguration('imu')
    odom_topic = LaunchConfiguration('odom')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    occ_grid_resolution = LaunchConfiguration('occ_grid_resolution')
    occ_grid_publish_period = LaunchConfiguration('occ_grid_publish_period')
    map_topic = LaunchConfiguration('map_topic')

    # Frames for static TFs
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('configuration_directory', default_value=default_config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=default_config_basename),
        DeclareLaunchArgument('points2', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('imu', default_value='/unilidar/imu'),
        DeclareLaunchArgument('odom', default_value='/Odometry'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('occ_grid_resolution', default_value='0.05'),
        DeclareLaunchArgument('occ_grid_publish_period', default_value='1.0'),
        DeclareLaunchArgument('map_topic', default_value='/Laser_map'),
        # Static TF defaults (edit to your actual frames)
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_frame', default_value='unilidar_imu'),
        DeclareLaunchArgument('lidar_frame', default_value='unilidar'),  # set to the actual frame_id of /unilidar/cloud
    ]

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('points2', points2_topic),
            ('imu', imu_topic),
            ('odom', odom_topic),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': occ_grid_resolution,
            'publish_period_sec': occ_grid_publish_period,
        }],
        remappings=[('map', map_topic)],
    )

    # Static TFs (zero offset/rotation; adjust if needed)
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, imu_frame],
        output='screen',
    )
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0', '0', '0', '0', base_frame, lidar_frame],
        output='screen',
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(declare_args + [
        static_tf_base_to_imu,
        static_tf_base_to_lidar,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])