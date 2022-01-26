#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''
def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('dwa_local_planner'),
            'param',
            'waypoints.yaml'))

    return LaunchDescription([

        ExecuteProcess(
            cmd=["ros2", "run", "turtlesim", "turtlesim_node"]
        ),
        ExecuteProcess(
            cmd=["ros2", "service", "call", "/spawn", "turtlesim/srv/Spawn", "{x: 1.0, y: 1.0, theta: 0, name: 'turtle2'}"]
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "dwa_local_planner", "dwa"]
        ),

    ])
'''
def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('dwa_local_planner'),
            'param',
            'waypoints.yaml'))

    return LaunchDescription([

        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            node_name='turtlesim_node',
            output='screen'),

        ExecuteProcess(
            cmd=["ros2", "service", "call", "/spawn", "turtlesim/srv/Spawn", "{x: 1.0, y: 1.0, theta: 0, name: 'turtle2'}"]
        ),

        Node(
            package='dwa_local_planner',
            node_executable='dwa_planner',
            node_name='dwa_planner',
            parameters=[param_dir],
            output='screen'),
    ])
