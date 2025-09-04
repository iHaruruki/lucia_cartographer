# Copyright 2019 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Config files
    pkg_share = get_package_share_directory('lucia_cartographer')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(pkg_share, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='unilidar_3d.lua'
    )

    # Topics (adjust defaults to your setup)
    points_topic = LaunchConfiguration('points_topic', default='/unilidar/cloud')
    imu_topic = LaunchConfiguration('imu_topic', default='/unilidar/imu')
    odom_topic = LaunchConfiguration('odom_topic', default='/Odometry')

    # Occupancy grid params
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # Toggles
    start_occupancy_grid = LaunchConfiguration('start_occupancy_grid', default='true')
    start_rviz = LaunchConfiguration('start_rviz', default='true')

    # RViz config
    rviz_config = os.path.join(pkg_share, 'rviz', '3d.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir,
                              description='Directory containing Cartographer .lua config'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename,
                              description='Cartographer .lua filename'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('points_topic', default_value=points_topic,
                              description='PointCloud2 topic for 3D LiDAR'),
        DeclareLaunchArgument('imu_topic', default_value=imu_topic,
                              description='IMU topic'),
        DeclareLaunchArgument('odom_topic', default_value=odom_topic,
                              description='Odometry topic'),
        DeclareLaunchArgument('resolution', default_value=resolution,
                              description='Occupancy grid resolution (meters/cell)'),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec,
                              description='OccupancyGrid publishing period (sec)'),
        DeclareLaunchArgument('start_occupancy_grid', default_value=start_occupancy_grid,
                              description='Start occupancy_grid_node'),
        DeclareLaunchArgument('start_rviz', default_value=start_rviz,
                              description='Start RViz'),

        # Cartographer node (3D)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('points2', points_topic),   # 3D LiDAR PointCloud2
                ('imu', imu_topic),          # IMU
                ('odom', odom_topic),        # Wheel odom (if available)
            ],
        ),

        # Publish 2D occupancy grid from Cartographer’s submaps (optional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            condition=IfCondition(start_occupancy_grid),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(start_rviz),
        ),
    ])