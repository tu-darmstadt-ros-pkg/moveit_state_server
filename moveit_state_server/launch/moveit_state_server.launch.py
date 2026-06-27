#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

folder_path = os.path.expanduser("~/.ros/moveit_state_server_data")


def launch_setup(context, *args, **kwargs):
    # Resolve launch arguments
    robot_name = LaunchConfiguration("robot_name").perform(context)
    state_server_yaml = LaunchConfiguration("state_server_config").perform(context)

    # Build MoveIt config package name (<robot>_moveit_config)
    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=f"{robot_name}_moveit_config"
    ).to_moveit_configs()

    # (Optional) launch the moveit_state_server itself with the same YAML
    state_server_node = Node(
        package="moveit_state_server",
        executable="moveit_state_server",
        # NOTE: intentionally no "name=" here. Setting it injects a global
        # "--remap __node:=..." argument that MoveGroupInterface's internally
        # created helper nodes (which honor global arguments) pick up, renaming
        # them to the same name and producing duplicate node names in the graph.
        # The executable already names itself "moveit_state_server" in code.
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            state_server_yaml,
            {"folder_path": folder_path, "use_sim_time": True},
        ],
    )

    return [state_server_node]


def generate_launch_description() -> LaunchDescription:
    # Path to the default YAML in the moveit_state_server package
    default_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_state_server"), "config", "state_server.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value=TextSubstitution(text="athena"),
                description="Robot base name (expects a '<robot_name>_moveit_config' package).",
            ),
            DeclareLaunchArgument(
                "state_server_config",
                default_value=default_yaml,
                description="YAML file with parameters for moveit_state_server and/or MoveGroup demo.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
