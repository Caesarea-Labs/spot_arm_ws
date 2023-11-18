#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: July, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

# irb120_simulation.launch.py:
# Launch file for the ABB-IRB120 Robot GAZEBO SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    # ***** GAZEBO ***** #
    # DECLARE Gazebo WORLD file:
    spot_gazebo = os.path.join(
        get_package_share_directory('spot_arm_gazebo'),
        'worlds',
        'spot_empty.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
         launch_arguments={'world': spot_gazebo}.items(),
    )

    # ========== COMMAND LINE ARGUMENTS ========== #
    print("")
    print(" --- Spot Arm Gazebo Simulation --- ")
    print("        (c) Cesarea Labs     ")
    print("")




    # ***** ROBOT DESCRIPTION ***** #
    # spot_arm Description file package:
    description_path = os.path.join(
        get_package_share_directory('spot_description'))
    # spot_arm ROBOT urdf file path:

    print("robot_description")


    robot_description_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_path, "urdf", "spot_control.urdf.xacro"]),
            " ",
        ])



    robot_description = {'robot_description': robot_description_config}



    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )





    # SPAWN ROBOT TO GAZEBO:
    print('spawn_entity')
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'spot',
                                   '-x', '1.0',
                                   '-z', '1.0',
                                   '-y', '1.0'],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # spot_arm_controller:
    # controller_manager_path = get_package_share_directory('controller_manager')
    # print(controller_manager_path)
    spot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["spot_arm_controller", "-c", "/controller_manager"],
    )
    # grip_controllers:
    grip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grip_controller", "-c", "/controller_manager"],
    )
    print("Entering the launch description")
    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        #
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[
        #             joint_state_broadcaster_spawner,
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[
        #             spot_arm_controller_spawner,
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=spot_arm_controller_spawner,
        #         on_exit=[
        #             grip_controller_spawner,
        #         ]
        #     )
        # ),
        gazebo,
        # node_robot_state_publisher,
        # spawn_entity,
    ])