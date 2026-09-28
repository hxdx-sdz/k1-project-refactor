from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([
            FindPackageShare("brain"), "config", "world.yaml"])),
        DeclareLaunchArgument("motion_config", default_value=PathJoinSubstitution([
            FindPackageShare("motion"), "config", "motion.yaml"])),
        DeclareLaunchArgument("start_motion", default_value="true"),
        DeclareLaunchArgument("team_id", default_value="8"),
        DeclareLaunchArgument("player_id", default_value="1"),
        Node(
            package="brain", executable="brain_node", output="screen",
            parameters=[LaunchConfiguration("config"), {
                "team_id": ParameterValue(LaunchConfiguration("team_id"), value_type=int),
                "player_id": ParameterValue(LaunchConfiguration("player_id"), value_type=int),
            }],
        ),
        Node(
            package="motion", executable="motion_node", output="screen",
            parameters=[LaunchConfiguration("motion_config")],
            condition=IfCondition(LaunchConfiguration("start_motion")),
        ),
    ])
