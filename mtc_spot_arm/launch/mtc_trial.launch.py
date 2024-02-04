from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm").to_dict()
    # MTC Demo node
    # moveit_config = (
    #     MoveItConfigsBuilder("arm")
    #     .robot_description(file_path="config/spot.urdf.xacro")
    #     .to_moveit_configs()
    # )
    pick_place_demo = Node(
        package="mtc_spot_arm",
        executable="mtc_trial",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])