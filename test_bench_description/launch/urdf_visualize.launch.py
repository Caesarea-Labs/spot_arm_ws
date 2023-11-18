import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import launch_ros
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'Socket.urdf'
    #xacro_file = "box_bot.xacro"
    package_description = "test_bench_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()
    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output="screen"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
        # parameters=[zero_pos]
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        # condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
    )

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node
        ]
    )