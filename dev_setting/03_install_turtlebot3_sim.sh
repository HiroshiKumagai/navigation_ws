#!/usr/bin/env bash
# TurtleBot3 + Gazebo (gz sim Harmonic) シミュレーション環境 (ROS 2 Jazzy 用)
set -euo pipefail

ROS_DISTRO=jazzy

if [[ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
    echo "ROS 2 ${ROS_DISTRO} が見つかりません。先に 02_install_ros2_jazzy.sh を実行してください。" >&2
    exit 1
fi

echo "=== TurtleBot3 / Gazebo パッケージのインストール ==="
sudo apt update
sudo apt install -y \
    "ros-${ROS_DISTRO}-turtlebot3" \
    "ros-${ROS_DISTRO}-turtlebot3-simulations" \
    "ros-${ROS_DISTRO}-turtlebot3-msgs"

echo "=== ~/.bashrc 設定 ==="
MODEL_LINE="export TURTLEBOT3_MODEL=burger"
if ! grep -q "TURTLEBOT3_MODEL" ~/.bashrc; then
    echo "${MODEL_LINE}" >> ~/.bashrc
fi

echo "完了: 新しいターミナルを開くか 'source ~/.bashrc' を実行してください。"
