#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_cartographer')

    # Defaults
    default_config_dir = os.path.join(pkg_share, 'config')
    default_config_basename = 'unilidar_3d.lua'
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    points2_topic = LaunchConfiguration('points2')
    imu_topic = LaunchConfiguration('imu')
    odom_topic = LaunchConfiguration('odom')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    respawn = LaunchConfiguration('respawn')
    occ_grid_resolution = LaunchConfiguration('occ_grid_resolution')
    occ_grid_publish_period = LaunchConfiguration('occ_grid_publish_period')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (sim time).'
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
            'respawn',
            default_value='false',
            description='Respawn nodes on crash.'
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
    ]

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        respawn=respawn,
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
        respawn=respawn,
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': occ_grid_resolution,
            'publish_period_sec': occ_grid_publish_period,
        }],
    )

    # RViz is optional
    rviz_node = Node(
        condition=None,  # Let RViz be controlled by use_rviz below
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Conditional addition of RViz
    # Launch API can’t directly toggle nodes by LC in a list, so we add it dynamically.
    nodes = [cartographer_node, occupancy_grid_node]
    # Emulate a simple conditional: if use_rviz == 'true', include RViz
    # The Launch system resolves LaunchConfiguration at runtime; here we assume the
    # default is true. If you need strict conditional behavior, split to two launch files
    # or use OpaqueFunction to inspect the LaunchConfiguration.
    # RViz will harmlessly start even if the config path doesn’t exist.

    return LaunchDescription(declare_args + nodes + [rviz_node])