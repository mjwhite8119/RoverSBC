#!/usr/bin/env python3

# Authors: Martin White

import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'rover.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('rover_description'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='gui', 
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'), 
        DeclareLaunchArgument(name='model', 
            default_value=urdf,
            description='Absolute path to robot urdf file'),       

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=[urdf]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        )

    ])
