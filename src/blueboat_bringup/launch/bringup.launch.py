from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
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
    lookup_csv = LaunchConfiguration("lookup_csv")
    thruster_lpf_alpha = LaunchConfiguration("thruster_lpf_alpha")

    enable_gps = LaunchConfiguration("enable_gps")
    enable_gps_anchor = LaunchConfiguration("enable_gps_anchor")
    gps_params_file = LaunchConfiguration("gps_params_file")

    thruster_lpf_alpha_default = _load_thruster_lpf_alpha()

    description_pkg = get_package_share_directory("blueboat_cirtesu_description")
    hardware_pkg = get_package_share_directory("sura_hardware_interface")
    bringup_pkg = get_package_share_directory("blueboat_bringup")

    xacro_file = os.path.join(description_pkg, "urdf", "blueboat.xacro")

    default_lookup_csv = os.path.join(
        hardware_pkg,
        "config",
        "m200_lookup.csv"
    )

    ros2_control_params_file = os.path.join(
        bringup_pkg,
        "config",
        "ros2_control_params.yaml"
    )

    default_gps_params_file = os.path.join(
        bringup_pkg,
        "config",
        "gps_params.yaml"
    )

    robot_description = Command([
        "xacro ",
        xacro_file,
        " environment:=", environment,
        " lookup_csv:=", lookup_csv
    ])

    gps_enabled = IfCondition(
        PythonExpression([
            "'", enable_gps, "' == 'true'"
        ])
    )

    real_and_gps_anchor_enabled = IfCondition(
        PythonExpression([
            "'", environment, "' == 'real' and '",
            enable_gps, "' == 'true' and '",
            enable_gps_anchor, "' == 'true'"
        ])
    )

    thruster_lpf_alpha_env = SetEnvironmentVariable(
        name="BLUEBOAT_THRUSTER_LPF_ALPHA",
        value=thruster_lpf_alpha
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_control_params_file,
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

    gps_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gps_broadcaster"],
        output="screen",
        condition=gps_enabled
    )

    battery_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["battery_broadcaster"],
        output="screen"
    )


    gps_anchor_node = Node(
        package="sura_sensors",
        executable="gps_anchor_node",
        name="gps_anchor_node",
        output="screen",
        parameters=[
            gps_params_file
        ],
        condition=real_and_gps_anchor_enabled
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "environment",
            default_value="real",
            description="Execution environment: real or sim"
        ),
        DeclareLaunchArgument(
            "lookup_csv",
            default_value=default_lookup_csv,
            description="Path to thruster lookup CSV"
        ),
        DeclareLaunchArgument(
            "thruster_lpf_alpha",
            default_value=str(thruster_lpf_alpha_default),
            description="Low-pass alpha for thruster commands in ThrustersSystem"
        ),
        DeclareLaunchArgument(
            "enable_gps",
            default_value="true",
            description="Spawn the GPS broadcaster"
        ),
        DeclareLaunchArgument(
            "enable_gps_anchor",
            default_value="false",
            description="Launch GPS anchor node when environment is real"
        ),
        DeclareLaunchArgument(
            "gps_params_file",
            default_value=default_gps_params_file,
            description="YAML file with GPS and GPS anchor parameters"
        ),

        thruster_lpf_alpha_env,

        ros2_control_node,

        thruster_test_spawner,
        body_force_spawner,
        body_velocity_spawner,
        body_position_spawner,
        imu_broadcaster_spawner,
        gps_broadcaster_spawner,
        battery_broadcaster_spawner,
        gps_anchor_node,
    ])
