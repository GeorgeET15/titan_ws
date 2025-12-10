# launch/titan_v1.launch.py

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for TITAN V1:
      - Starts Gazebo Harmonic via ros_gz_sim
      - Spawns the TITAN robot from Xacro description
      - Runs robot_state_publisher
      - Runs ros_gz_bridge using config/gz_bridge.yaml
      - Starts teleop_twist_keyboard for manual testing (cmd_vel)
    """

    pkg_name = "titan_v1"

    # =========================
    #      Launch arguments
    # =========================

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="World SDF file to load with Gazebo Sim (relative to ros_gz_sim share or absolute path).",
    )

    # =========================
    #       File paths
    # =========================

    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, "model", "titan_v1.xacro")
    gz_bridge_file = os.path.join(pkg_share, "config", "gz_bridge.yaml")

    # Process Xacro into robot_description XML string
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # =========================
    #      Gazebo (ros_gz_sim)
    # =========================

    # This uses the standard ros_gz_sim gz_sim.launch.py:
    #   gz sim -r -v4 <world>
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", LaunchConfiguration("world")],
        }.items(),
    )

    # =========================
    #        Spawn robot
    # =========================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_titan_v1",
        output="screen",
        arguments=[
            "-name",
            "titan_v1",
            "-string",
            robot_description_xml,
            "-z",
            "0.1",  # small offset above ground to avoid collision glitches
        ],
    )

    # =========================
    #  Robot State Publisher
    # =========================

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_xml,
                "use_sim_time": True,
            }
        ],
    )

    # =========================
    #      ROS <-> GZ Bridge
    # =========================

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={gz_bridge_file}",
        ],
    )

    # =========================
    #      Teleop (cmd_vel)
    # =========================

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        # This opens an xterm so you can drive with WASD keys.
        # If you don't have xterm, remove the 'prefix' line.
        prefix="xterm -e",
        output="screen",
        remappings=[
            ("cmd_vel", "cmd_vel"),
        ],
    )

    # =========================
    #   Launch description
    # =========================

    return LaunchDescription(
        [
            world_arg,
            gazebo,
            spawn_robot,
            state_publisher,
            bridge,
            teleop,
        ]
    )
