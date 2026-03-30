from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    environment = LaunchConfiguration("environment")

    description_pkg = get_package_share_directory("blueboat_cirtesu_description")
    hardware_pkg = get_package_share_directory("thrusters_hardware_interface")
    bringup_pkg = get_package_share_directory("blueboat_bringup")

    xacro_file = os.path.join(description_pkg, "urdf", "blueboat.xacro")
    csv_file = os.path.join(hardware_pkg, "config", "t200_lookup.csv")
    params_file = os.path.join(bringup_pkg, "config", "ros2_control_params.yaml")

    robot_description = Command([
        "xacro ",
        xacro_file,
        " environment:=", environment,
        " lookup_csv:=", csv_file
    ])

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

    return LaunchDescription([
        DeclareLaunchArgument(
            "environment",
            default_value="real",
            description="Execution environment: real or sim"
        ),
        ros2_control_node,
        thruster_test_spawner,
        body_velocity_spawner,
        body_force_spawner
    ])
