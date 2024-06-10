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

def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = [
        launch.actions.DeclareLaunchArgument("arm", default_value="false", description="include arm in robot model"),
        launch.actions.DeclareLaunchArgument(
            "tf_prefix", default_value='""', description="apply namespace prefix to robot links and joints"),
    ]

    return launch.LaunchDescription(launch_arguments + [launch.actions.OpaqueFunction(function=launch_setup)])





def generate_launch_description():
    spot_gazebo = os.path.join(
        get_package_share_directory('spot_arm_gazebo'),
        'worlds',
        'spot_empty.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': spot_gazebo}.items(),
    )

    package_description_path=os.path.join(get_package_share_directory('spot_description'))

    # ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    #     output='screen'),

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([package_description_path, "urdf", "spot_control.urdf.xacro"]),
            " ",
            # "arm:=true",
            # LaunchConfiguration("arm"),
            # " ",
            # "tf_prefix:=",
            # LaunchConfiguration("tf_prefix"),
            # " ",
        ])
    params= {'robot_description': robot_description}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control' , 'load_controller','--set-state','active','joint_state_broadcaster'],
        output='screen'
    )
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'spot_arm_controller'],
        output='screen'
    )

    load_grip_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'grip_controller'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description',
                   '-entity','spot',
                   '-x', '0.0',
                   '-z', '1.0',
                   '-y', '1.1',
                   '-Y', '-1.57'
                   ],
        # world= os.path.join(package_description_path, "worlds/outdoor.world"),
        output='screen')
    sdf_model=os.path.join(get_package_share_directory('spot_arm_gazebo'),
        'models','12gen',
        'model.sdf')
    spawn_chip = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        name="spawn_sdf_entity",
                        arguments=['-entity', 'Chip', '-file',sdf_model,
                                   '-x', str(random.gauss(0,0.1)),
                                   '-y', str(random.gauss(0,0.1)),
                                   '-z', '0.71',
                                   '-R', '0.0',
                                   '-P', '0.0',
                                   '-Y', str(random.uniform(-0.2,0))
                                   ], output='screen')
    table_model = os.path.join(get_package_share_directory('spot_arm_gazebo'),
                             'models', 'table',
                             'model.sdf')
    spawn_table = Node(package='gazebo_ros',
                      executable='spawn_entity.py',
                      name="spawn_table_entity",
                      arguments=['-entity', 'Table', '-file', table_model,
                                 '-x', '0',
                                 '-y', '0',
                                 '-z', '0'
                                ],
                      output='screen')

    socket_description_path = os.path.join(get_package_share_directory('test_bench_description'))
    socket_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([socket_description_path, "urdf", "Socket.urdf"]),
            " ",
        ])
    params = {'robot_description': socket_description}

    node_socket_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='socket_state_publisher',
        output='screen',
        parameters=[params],
        remappings=[('/robot_description', '/socket_description')]
    )

    spawn_socket = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/socket_description',
                   '-entity', 'socket',
                   '-x', '0.1',
                   '-z', '0.71',
                   '-y', '0.1',
                   '-R', '1.57',
                   '-Y', '1.57'
                   ],
        output='screen')





    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_table],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_table,
                on_exit=[spawn_chip],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_chip,
                on_exit=[node_socket_state_publisher],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_chip,
                on_exit=[spawn_socket],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_grip_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        load_joint_state_controller,
        spawn_entity,
    ])