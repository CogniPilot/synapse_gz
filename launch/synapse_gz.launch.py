import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('host', default_value='127.0.0.1',
                          description='port for gz sim'),
    DeclareLaunchArgument('port', default_value='4243',
                          description='port for cerebri'),
    DeclareLaunchArgument('vehicle', default_value='b3rb',
                          description='vehicle'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('gyro_bias_z', default_value='0.0',
                          description='gyro_bias_z in rad/s'),
]


def generate_launch_description():

    # Launch configurations
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    vehicle = LaunchConfiguration('vehicle')

    synapse_gz = Node(
        package='synapse_gz',
        executable='synapse_gz',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle': LaunchConfiguration('vehicle'),
            'gyro_bias_z': LaunchConfiguration('gyro_bias_z'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(synapse_gz)
    return ld

