from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("model", default_value="Bar3XT"),
        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument("frame_id", default_value="barxt_link"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("fluid_density_kg_m3", default_value="997.0"),
        DeclareLaunchArgument("surface_pressure_bar", default_value="1.01325"),
    ]

    node = Node(
        package="barxt_ros2",
        executable="barxt_node",
        name="barxt_node",
        output="screen",
        parameters=[
            {
                "model": LaunchConfiguration("model"),
                "i2c_bus": LaunchConfiguration("i2c_bus"),
                "i2c_address": LaunchConfiguration("i2c_address"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "fluid_density_kg_m3": LaunchConfiguration("fluid_density_kg_m3"),
                "surface_pressure_bar": LaunchConfiguration("surface_pressure_bar"),
            }
        ],
    )

    return LaunchDescription(arguments + [node])

