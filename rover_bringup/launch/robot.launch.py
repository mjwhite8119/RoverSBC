#!/usr/bin/env python3
#
# Authors: Martin White

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')

    tb3_param_dir = LaunchConfiguration(
        'rover_param_dir',
        default=os.path.join(
            get_package_share_directory('rover_bringup'),
            'param',
            'rover.yaml'))

    # lidar_pkg_dir = LaunchConfiguration(
    #     'lidar_pkg_dir',
    #     default=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with ESP32'),

        DeclareLaunchArgument(
            'rover_param_dir',
            default_value=tb3_param_dir,
            description='Full path to rover parameter file to load'),

        # Add the State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/rover_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Add the Lidar
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([lidar_pkg_dir, '/ydlidar_launch.py']),
        #     launch_arguments={}.items(),
        # ),

        # Add tf2 broadcaster
        Node(
            package='rover_node',
            executable='rover_tf2_broadcaster',
            name='rover_tf2_broadcaster_node'
        ),

        # Add the rover node
        Node(
            package='rover_node',
            executable='rover_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])
