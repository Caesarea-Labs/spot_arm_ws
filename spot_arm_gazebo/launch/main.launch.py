from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():
    # Get the directory of the package containing the launch files
    gazebo_dir = get_package_share_directory('spot_arm_gazebo')
    moveit_config_dir = get_package_share_directory('arm_moveit_config')
    move_service_node = Node(
        package='move_arm',
        executable='move_arm_service',
        name='move_service_node',
        output='screen'
    )
    yolo_node = Node(
        package='yolo',
        executable='image_node',
        name='move_service_node',
        output='screen'
    )
    SetLaunchConfiguration('use_sim_time', 'true')
    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'gazebo.launch.py')),
            # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'demo.launch.py')),
            # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )
    kill_rviz = ExecuteProcess(
        cmd=['killall', '-9', 'rviz2'],
        output='screen'
    )

    sleep5=ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )
    sleep10 = ExecuteProcess(
        cmd=['sleep', '10'],
        output='screen'
    )

    return LaunchDescription([

        gazebo_launch,
        sleep5,
        sleep10,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep5,
                on_exit=[move_group_launch],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep10,
                on_exit=[move_service_node, yolo_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep10,
                on_exit=[kill_rviz],
            )
        ),



    ])
