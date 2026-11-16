"""Run the demo on the running simulation: homing, then pick-and-place cycles, with logging.

    ros2 launch robotic_arm_control demo.launch.py                    (4 cycles, both colours)
    ros2 launch robotic_arm_control demo.launch.py cycles:=2 record:=true   (also saves screenshots and pick_place.mp4)
    ros2 launch robotic_arm_control demo.launch.py estop_test:=true   (pick one cube, hold it, press the E-stop)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("cycles", default_value="4"),
        DeclareLaunchArgument("homing", default_value="true"),
        DeclareLaunchArgument("estop_test", default_value="false"),
        DeclareLaunchArgument("record", default_value="false", description="save gazebo_world.png, gazebo_arm.png and pick_place.mp4"),
        DeclareLaunchArgument("video_seconds", default_value="50.0"),
        Node(package="robotic_arm_control", executable="logger_node", output="screen", parameters=[sim]),
        Node(package="robotic_arm_control", executable="camera_capture", output="screen", condition=IfCondition(LaunchConfiguration("record")),
             parameters=[sim, {"video_seconds": ParameterValue(LaunchConfiguration("video_seconds"), value_type=float)}]),
        Node(package="robotic_arm_control", executable="pick_place_node", output="screen", on_exit=Shutdown(),
             parameters=[sim, {"cycles": ParameterValue(LaunchConfiguration("cycles"), value_type=int),
                               "homing": ParameterValue(LaunchConfiguration("homing"), value_type=bool),
                               "estop_test": ParameterValue(LaunchConfiguration("estop_test"), value_type=bool)}]),
    ])
