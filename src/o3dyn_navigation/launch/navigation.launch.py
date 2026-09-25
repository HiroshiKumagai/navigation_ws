"""Isaac Sim 上の O3dyn で Nav2 を動かす起動ファイル。

先に Isaac Sim 側（sim/o3dyn_bringup.py）を起動しておくこと。
    ros2 launch o3dyn_navigation navigation.launch.py [map:=<map.yaml>] [rviz:=true] [joy:=true] [mode:=navigation|joystick]

起動するもの:
    - base_link -> base_scan の静的 TF（統合スキャンの基準フレーム。LiDAR と同じ高さ）
    - scan_merger（/scan_front_left, /scan_rear_right -> /scan）
    - Nav2 一式（map_server, AMCL, 経路計画・追従, velocity_smoother, collision_monitor ほか）
    - cmd_vel_switch（/cmd_vel_smoothed と /cmd_vel_joy の一方を /cmd_vel_switched へ。collision_monitor の手前）
    - joy_node + teleop_twist_joy（ゲームパッド -> /cmd_vel_joy。joy:=false で起動しない）
    - rviz2（Nav2 標準の表示設定）

速度指令の流れ:
    Nav2 -> velocity_smoother -> /cmd_vel_smoothed --+
                                                      +-> cmd_vel_switch -> /cmd_vel_switched -> collision_monitor -> /cmd_vel
    ゲームパッド -> joy -> teleop_twist_joy -> /cmd_vel_joy -+
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR_HEIGHT = "0.25"  # [m] sim/o3dyn_bringup.py の LiDAR 取り付け高さ


def generate_launch_description():
    pkg_dir = get_package_share_directory("o3dyn_navigation")
    nav2_dir = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    joy = LaunchConfiguration("joy")
    teleop_params = os.path.join(pkg_dir, "config", "teleop_f310.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=os.path.join(pkg_dir, "maps", "warehouse.yaml")),
            DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_dir, "config", "nav2_params.yaml")),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("joy", default_value="true"),
            DeclareLaunchArgument("mode", default_value="navigation", description="navigation または joystick"),
            Node(
                package="o3dyn_navigation",
                executable="cmd_vel_switch",
                name="cmd_vel_switch",
                parameters=[{"use_sim_time": True, "initial_mode": LaunchConfiguration("mode")}],
                remappings=[
                    ("nav_cmd_vel", "cmd_vel_smoothed"),
                    ("joy_cmd_vel", "cmd_vel_joy"),
                    ("ui_cmd_vel", "cmd_vel_ui"),
                    ("cmd_vel_out", "cmd_vel_switched"),
                ],
                output="screen",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[teleop_params],
                condition=IfCondition(joy),
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[teleop_params],
                remappings=[("cmd_vel", "cmd_vel_joy")],
                condition=IfCondition(joy),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_scan_tf",
                arguments=["--z", LIDAR_HEIGHT, "--frame-id", "base_link", "--child-frame-id", "base_scan"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="o3dyn_navigation",
                executable="scan_merger",
                name="scan_merger",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "bringup_launch.py")),
                launch_arguments={
                    "map": map_yaml,
                    "params_file": params_file,
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "rviz_launch.py")),
                launch_arguments={"use_sim_time": "true"}.items(),
                condition=IfCondition(rviz),
            ),
        ]
    )
