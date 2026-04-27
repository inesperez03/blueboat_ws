from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_thruster_lpf_alpha():
    params_file = os.path.join(
        get_package_share_directory("blueboat_bringup"),
        "config",
        "ros2_control_params.yaml"
    )
    try:
        with open(params_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        return data.get("thrusters_system", {}).get("ros__parameters", {}).get(
            "thruster_lpf_alpha", 0.25
        )
    except Exception:
        return 0.25


def generate_launch_description():
    environment = LaunchConfiguration("environment")
    thruster_lpf_alpha = LaunchConfiguration("thruster_lpf_alpha")
    thruster_lpf_alpha_default = _load_thruster_lpf_alpha()

    description_pkg = get_package_share_directory("blueboat_cirtesu_description")
    hardware_pkg = get_package_share_directory("sura_hardware_interface")
    bringup_pkg = get_package_share_directory("blueboat_bringup")

    xacro_file = os.path.join(description_pkg, "urdf", "blueboat.xacro")
    csv_file = os.path.join(hardware_pkg, "config", "m200_lookup.csv")
    params_file = os.path.join(bringup_pkg, "config", "ros2_control_params.yaml")

    robot_description = Command([
        "xacro ",
        xacro_file,
        " environment:=", environment,
        " lookup_csv:=", csv_file
    ])

    thruster_lpf_alpha_env = SetEnvironmentVariable(
        name="BLUEBOAT_THRUSTER_LPF_ALPHA",
        value=thruster_lpf_alpha
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            params_file,
            {"robot_description": robot_description}
        ],
        output="screen"
    )

    thruster_test_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["thruster_test_controller", "--inactive"],
        output="screen"
    )

    body_force_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_force_controller", "--inactive"],
        output="screen"
    )

    body_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_velocity_controller", "--inactive"],
        output="screen"
    )

    body_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_position_controller", "--inactive"],
        output="screen"
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--inactive"],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "environment",
            default_value="real",
            description="Execution environment: real or sim"
        ),
        DeclareLaunchArgument(
            "thruster_lpf_alpha",
            default_value=str(thruster_lpf_alpha_default),
            description="Low-pass alpha for thruster commands in ThrustersSystem"
        ),
        thruster_lpf_alpha_env,
        ros2_control_node,
        thruster_test_spawner,
        body_velocity_spawner,
        body_position_spawner,
        imu_broadcaster_spawner,
        body_force_spawner
    ])
