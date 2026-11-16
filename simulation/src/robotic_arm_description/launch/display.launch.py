"""RViz view of the cleaned URDF with joint sliders (replaces the ROS 1 display.launch).

    ros2 launch robotic_arm_description display.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([FindPackageShare("robotic_arm_description"), "urdf", "robotic_arm.urdf.xacro"])
    description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["--fixed-frame", "world"]),
    ])
