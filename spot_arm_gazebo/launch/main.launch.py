from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():
    # Get the directory of the package containing the launch files
    close_rviz = LaunchConfiguration('close_rviz')
    show_detection_flag = LaunchConfiguration('show_detection')

    kill_rviz_launch_arg = DeclareLaunchArgument(
        'close_rviz',
        default_value='true',
        description='Flag to kill rviz'
    )

    show_detection_launch_arg = DeclareLaunchArgument(
        'show_detection_flag',
        default_value='true',
        description='Flag to show detection results'
    )

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
    fix_socket = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/ATTACHLINK', 'linkattacher_msgs/srv/AttachLink',"{model1_name: 'Table', link1_name: 'link', model2_name: 'socket', link2_name: 'pcb'}"],
        output='screen'
    )

    detach_socket = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/DETACHLINK', 'linkattacher_msgs/srv/AttachLink',
             "{model1_name: 'Table', link1_name: 'link', model2_name: 'socket', link2_name: 'pcb'}"],
        output='screen'
    )

    fix_spot = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/ATTACHLINK', 'linkattacher_msgs/srv/AttachLink',
             "{model1_name: 'Table', link1_name: 'link', model2_name: 'spot', link2_name: 'body'}"],
        output='screen'
    )

    detach_spot = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/DETACHLINK', 'linkattacher_msgs/srv/AttachLink',
             "{model1_name: 'Table', link1_name: 'link', model2_name: 'spot', link2_name: 'body'}"],
        output='screen'
    )

    kill_rviz = ExecuteProcess(
        cmd=['killall', '-9', 'rviz2'],
        condition = IfCondition(close_rviz),
        output='screen'
    )

    show_detection = ExecuteProcess(
        cmd=['ros2', 'run', 'image_tools', 'showimage', '--ros-args', '--remap', '/image:=/detection_results'],
        condition = IfCondition(show_detection_flag),
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

    sleep15 = ExecuteProcess(
        cmd=['sleep', '15'],
        output='screen'
    )
    sleep25 = ExecuteProcess(
        cmd=['sleep', '25'],
        output='screen'
    )

    return LaunchDescription([
        kill_rviz_launch_arg,
        show_detection_launch_arg,
        gazebo_launch,
        sleep5,
        sleep10,
        sleep15,
        sleep25,
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep15,
                on_exit=[fix_spot , fix_socket],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep25,
                on_exit=[show_detection],
            )
        ),



    ])
