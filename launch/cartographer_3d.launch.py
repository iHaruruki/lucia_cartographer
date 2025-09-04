#!/usr/bin/env python3
# Launch Cartographer 3D with overridable config and topics.
#
# Examples:
#   ros2 launch lucia_cartographer cartographer_3d.launch.py \
#     configuration_directory:=/home/robot/ros2_ws/src/lucia_cartographer/config \
#     configuration_basename:=unilidar_3d.lua \
#     points2:=/unilidar/cloud imu:=/unilidar/imu odom:=/Odometry
#
# To publish occupancy grid as /Laser_map (default here), remap if needed:
#   ros2 launch lucia_cartographer cartographer_3d.launch.py map_topic:=/map

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_cartographer')

    # Defaults
    default_config_dir = os.path.join(pkg_share, 'config')
    default_config_basename = 'unilidar_3d.lua'
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    points2_topic = LaunchConfiguration('points2')   # FIXED
    imu_topic = LaunchConfiguration('imu')           # FIXED
    odom_topic = LaunchConfiguration('odom')         # FIXED
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    occ_grid_resolution = LaunchConfiguration('occ_grid_resolution')
    occ_grid_publish_period = LaunchConfiguration('occ_grid_publish_period')
    map_topic = LaunchConfiguration('map_topic')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (simulation time).'
        ),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=default_config_dir,
            description='Directory that contains the Cartographer Lua config.'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=default_config_basename,
            description='Lua configuration file name.'
        ),
        DeclareLaunchArgument(
            'points2',
            default_value='/unilidar/cloud',
            description='PointCloud2 topic for 3D lidar.'
        ),
        DeclareLaunchArgument(
            'imu',
            default_value='/unilidar/imu',
            description='IMU topic.'
        ),
        DeclareLaunchArgument(
            'odom',
            default_value='/Odometry',
            description='Odometry topic.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz2 config file.'
        ),
        DeclareLaunchArgument(
            'occ_grid_resolution',
            default_value='0.05',
            description='Occupancy grid resolution in meters.'
        ),
        DeclareLaunchArgument(
            'occ_grid_publish_period',
            default_value='1.0',
            description='Occupancy grid publish period in seconds.'
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/Laser_map',
            description='OccupancyGrid output topic (remap from "map").'
        ),
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
        remappings=[
            ('map', map_topic),
        ],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(declare_args + [cartographer_node, occupancy_grid_node, rviz_node])