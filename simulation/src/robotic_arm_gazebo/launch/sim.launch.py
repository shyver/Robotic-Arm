"""Start the simulation: Gazebo Fortress + arm + ros2_control controllers + E-stop service.

    ros2 launch robotic_arm_gazebo sim.launch.py                 (with the Gazebo window)
    ros2 launch robotic_arm_gazebo sim.launch.py headless:=true  (server only, no window)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robotic_arm_control import planner
from robotic_arm_control.kinematics import ARM_JOINTS


def generate_launch_description():
    desc = get_package_share_directory("robotic_arm_description")
    ctrl = get_package_share_directory("robotic_arm_control")
    gaz = get_package_share_directory("robotic_arm_gazebo")
    world = os.path.join(gaz, "worlds", "sorting_cell.sdf")
    controllers = os.path.join(ctrl, "config", "controllers.yaml")

    # start pose = balanced pose (gravity torque of J2..J4 = 0): the arm does not move before the controllers are active
    _, _, poses = planner.load_all()
    init = [f" initial_{n}:={q:.5f}" for n, q in zip(ARM_JOINTS, poses.start)]
    description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "robotic_arm.urdf.xacro"), " controllers_file:=", controllers] + init),
        value_type=str)

    # package://robotic_arm_description/... is resolved by Gazebo through this path (parent of the package share dir)
    resource = os.pathsep.join([os.path.dirname(desc), os.path.join(gaz, "worlds"), os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")])
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "robotic_arm", "-x", "0", "-y", "0", "-z", "0"])

    def spawner(name, *extra):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60", *extra])

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("stepper_controller")
    servo = spawner("servo_controller")
    grip = spawner("gripper_controller")
    estop_ctrl = spawner("estop_effort_controller", "--inactive")

    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                   "/cell_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image"]
    for i in range(1, 5):
        bridge_args += [f"/grasp/cube_{i}/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
                        f"/grasp/cube_{i}/detach@std_msgs/msg/Empty]ignition.msgs.Empty"]

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="true: server only (ign gazebo -s)"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch), launch_arguments={"gz_args": f"-r -v 2 {world}"}.items(),
                                 condition=UnlessCondition(LaunchConfiguration("headless"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch), launch_arguments={"gz_args": f"-r -s -v 2 {world}"}.items(),
                                 condition=IfCondition(LaunchConfiguration("headless"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=bridge_args, output="screen"),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, arm, servo, grip])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[estop_ctrl])),
        Node(package="robotic_arm_control", executable="estop_node", output="screen", parameters=[{"use_sim_time": True}]),
    ])
