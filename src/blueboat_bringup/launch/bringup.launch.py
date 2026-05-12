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
    enable_diagnostics = LaunchConfiguration("enable_diagnostics")
    enable_status_light = LaunchConfiguration("enable_status_light")
    enable_world_enu_identity_tf = LaunchConfiguration("enable_world_enu_identity_tf")
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

    real_and_diagnostics_enabled = IfCondition(
        PythonExpression([
            "'", environment, "' == 'real' and '",
            enable_diagnostics, "' == 'true'"
        ])
    )

    real_and_status_light_enabled = IfCondition(
        PythonExpression([
            "'", environment, "' == 'real' and '",
            enable_status_light, "' == 'true'"
        ])
    )

    world_enu_identity_enabled = IfCondition(
        PythonExpression([
            "'", enable_world_enu_identity_tf, "' == 'true' and not ('",
            environment, "' == 'real' and '",
            enable_gps_anchor, "' == 'true')"
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

    status_light_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["status_light_controller"],
        output="screen",
        condition=real_and_status_light_enabled
    )

    world_enu_identity_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_enu_to_map_identity",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "world_enu", "map"
        ],
        output="screen",
        condition=world_enu_identity_enabled
    )

    battery_diagnostics_node = Node(
        package="sura_diagnostics",
        executable="battery_status_diagnostics",
        name="battery_status_diagnostics",
        output="screen",
        parameters=[{
            "battery_topic": "/sura/sensors/battery",
        }],
        condition=real_and_diagnostics_enabled
    )

    gps_diagnostics_node = Node(
        package="sura_diagnostics",
        executable="gps_diagnostics",
        name="gps_diagnostics",
        output="screen",
        parameters=[{
            "sensor_topic": "/sura/sensors/gps/fix",
            "frequency_topic": "/sura/sensors/gps/fix",
        }],
        condition=real_and_diagnostics_enabled
    )

    hardware_components_diagnostics_node = Node(
        package="sura_diagnostics",
        executable="hardware_components_diagnostics",
        name="hardware_components_diagnostics",
        output="screen",
        parameters=[{
            "list_hardware_components_service": "/controller_manager/list_hardware_components",
            "component_names": ["SensorsSystem", "ThrustersSystem"],
            "diagnostic_names": ["/Hardware/Sensors", "/Hardware/Thrusters"],
        }],
        condition=real_and_diagnostics_enabled
    )

    catamaran_navigation_diagnostics_node = Node(
        package="sura_diagnostics",
        executable="catamaran_navigation_diagnostics",
        name="catamaran_navigation_diagnostics",
        output="screen",
        parameters=[{
            "world_frame": "world_enu",
            "position_frame": "blueboat/base_link_enu",
            "center_x": 0.0,
            "center_y": 0.0,
            "max_radius": 20.0,
            "radius_warning_margin": 2.0,
        }],
        condition=real_and_diagnostics_enabled
    )

    status_light_feedback_node = Node(
        package="blueboat_bringup",
        executable="status_light_feedback",
        name="status_light_feedback",
        output="screen",
        parameters=[{
            "diagnostics_topic": "/diagnostics",
            "commands_topic": "/status_light_controller/commands",
        }],
        condition=real_and_status_light_enabled
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
            "enable_diagnostics",
            default_value="true",
            description="Launch robot-side diagnostics nodes when environment is real"
        ),
        DeclareLaunchArgument(
            "enable_status_light",
            default_value="true",
            description="Launch the status light controller and feedback node when environment is real"
        ),
        DeclareLaunchArgument(
            "enable_world_enu_identity_tf",
            default_value="true",
            description="Publish identity TF world_enu->map when GPS anchor is not active"
        ),
        DeclareLaunchArgument(
            "gps_params_file",
            default_value=default_gps_params_file,
            description="YAML file with GPS and GPS anchor parameters"
        ),

        thruster_lpf_alpha_env,

        ros2_control_node,
        world_enu_identity_tf,

        thruster_test_spawner,
        body_force_spawner,
        body_velocity_spawner,
        body_position_spawner,
        imu_broadcaster_spawner,
        gps_broadcaster_spawner,
        battery_broadcaster_spawner,
        status_light_controller_spawner,
        battery_diagnostics_node,
        gps_diagnostics_node,
        hardware_components_diagnostics_node,
        catamaran_navigation_diagnostics_node,
        status_light_feedback_node,
        gps_anchor_node,
    ])
