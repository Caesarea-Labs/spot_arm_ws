import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import launch
import random
import xacro

# def generate_launch_description() -> launch.LaunchDescription:
#     launch_arguments = [
#         launch.actions.DeclareLaunchArgument("arm", default_value="false", description="include arm in robot model"),
#         launch.actions.DeclareLaunchArgument(
#             "tf_prefix", default_value='""', description="apply namespace prefix to robot links and joints"),
#     ]
#
#     return launch.LaunchDescription(launch_arguments + [launch.actions.OpaqueFunction(function=launch_setup)])





def generate_launch_description():


    package_description_path=os.path.join(get_package_share_directory('test_bench_description'))

    socket_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([package_description_path, "urdf", "Socket.urdf"]),
            " ",
        ])
    params= {'robot_description': socket_description}

    node_socket_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='socket_state_publisher',
        output='screen',
        parameters=[params],
        remappings=[('/robot_description','/socket_description')]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/socket_description',
                   '-entity','socket',
                   '-x', '0.1',
                   '-z', '1.1',
                   '-y', '0.1',
                   '-R','1.57',
                   '-Y','1.57'
                   ],
        output='screen')

    return LaunchDescription([
        node_socket_state_publisher,
        spawn_entity,

    ])