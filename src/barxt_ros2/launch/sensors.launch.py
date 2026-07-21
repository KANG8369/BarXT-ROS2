"""Launch the BarXT I2C node together with a MicroStrain IMU driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start the pressure and IMU sensor drivers with independent settings."""
    barxt_launch_file = os.path.join(
        get_package_share_directory("barxt_ros2"), "launch", "barxt.launch.py"
    )
    microstrain_launch_file = os.path.join(
        get_package_share_directory("microstrain_inertial_driver"),
        "launch",
        "microstrain_launch.py",
    )

    arguments = [
        DeclareLaunchArgument("model", default_value="Bar3XT"),
        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument("barxt_frame_id", default_value="barxt_link"),
        DeclareLaunchArgument("barxt_publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("fluid_density_kg_m3", default_value="997.0"),
        DeclareLaunchArgument("surface_pressure_bar", default_value="1.01325"),
        DeclareLaunchArgument(
            "imu_params_file",
            default_value="/ws/config/gv7_imu.yaml",
            description="ROS 2 parameter YAML passed to microstrain_inertial_driver.",
        ),
    ]

    barxt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(barxt_launch_file),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "i2c_bus": LaunchConfiguration("i2c_bus"),
            "i2c_address": LaunchConfiguration("i2c_address"),
            "frame_id": LaunchConfiguration("barxt_frame_id"),
            "publish_rate_hz": LaunchConfiguration("barxt_publish_rate_hz"),
            "fluid_density_kg_m3": LaunchConfiguration("fluid_density_kg_m3"),
            "surface_pressure_bar": LaunchConfiguration("surface_pressure_bar"),
        }.items(),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(microstrain_launch_file),
        launch_arguments={"params_file": LaunchConfiguration("imu_params_file")}.items(),
    )

    return LaunchDescription(arguments + [barxt, imu])
