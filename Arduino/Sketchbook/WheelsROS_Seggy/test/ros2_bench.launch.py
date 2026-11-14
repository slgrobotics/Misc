"""
Launch Seggy's real hardware with a direct wheel-velocity controller.

# Terminal 1:
ros2 launch ./test/ros2_bench.launch.py

# Terminal 2, once the controllers are active:
python3 test/ros2_bench.py

"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    seggy = Path.home() / "robot_ws/src/articubot_one/robots/seggy"
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            LaunchConfiguration("robot_xacro"), " sim_mode:=false hardware_config:=",
            LaunchConfiguration("hardware_config"),
        ]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_xacro", default_value=str(seggy / "description/robot.urdf.xacro")),
        DeclareLaunchArgument("hardware_config", default_value=str(seggy / "config/hardware.yaml")),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace="seggy_bench", parameters=[{"robot_description": description}],
        ),
        Node(
            package="controller_manager", executable="ros2_control_node",
            namespace="seggy_bench",
            parameters=[str(Path(__file__).with_name("ros2_bench.yaml"))],
        ),
        Node(
            package="controller_manager", executable="spawner",
            namespace="seggy_bench",
            arguments=[
                "joint_states", "wheel_velocity", "-c", "/seggy_bench/controller_manager",
                "--controller-manager-timeout", "20",
            ],
        ),
    ])
