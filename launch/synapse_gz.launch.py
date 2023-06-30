import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('host', default_value='127.0.0.1',
                          description='port for gz sim'),
    DeclareLaunchArgument('port', default_value='4241',
                          description='tcp port for cerebri'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():

    # Launch configurations
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    synapse_gz = Node(
        package='synapse_gz',
        executable='synapse_gz',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(synapse_gz)
    return ld

