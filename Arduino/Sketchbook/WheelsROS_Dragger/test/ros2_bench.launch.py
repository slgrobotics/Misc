"""Launch Dragger's real hardware with a direct wheel-velocity controller."""

from pathlib import Path

from ament_index_python.resources import get_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_driver_overlay(context):
    try:
        get_resource("hardware_interface__pluginlib__plugin", "diffdrive_arduino")
    except LookupError as error:
        raise RuntimeError(
            "diffdrive_arduino hardware plugin is missing from the sourced ROS overlays. "
            "Run 'source ~/tmp_ws/install/local_setup.bash' in this terminal and retry."
        ) from error
    return []


def generate_launch_description():
    dragger = Path.home() / "robot_ws/src/articubot_one/robots/dragger"
    controller_params = str(Path(__file__).resolve().with_name("ros2_bench.yaml"))
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            LaunchConfiguration("robot_xacro"), " sim_mode:=false hardware_config:=",
            LaunchConfiguration("hardware_config"),
        ]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_xacro", default_value=str(dragger / "description/robot.urdf.xacro")),
        DeclareLaunchArgument("hardware_config", default_value=str(dragger / "config/hardware.yaml")),
        OpaqueFunction(function=check_driver_overlay),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace="dragger_bench", parameters=[{"robot_description": description}],
        ),
        Node(
            package="controller_manager", executable="ros2_control_node",
            namespace="dragger_bench",
            parameters=[controller_params],
        ),
        Node(
            package="controller_manager", executable="spawner",
            namespace="dragger_bench",
            arguments=[
                "joint_states", "wheel_velocity", "-c", "/dragger_bench/controller_manager",
                "--param-file", controller_params,
                "--controller-manager-timeout", "20",
            ],
        ),
    ])
