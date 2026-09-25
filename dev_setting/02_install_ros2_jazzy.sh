#!/usr/bin/env bash
# ROS 2 Jazzy インストール (Ubuntu 24.04 Noble 用)
# 参考: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
set -euo pipefail

ROS_DISTRO=jazzy

. /etc/os-release
if [[ "${UBUNTU_CODENAME:-}" != "noble" ]]; then
    echo "ROS 2 Jazzy は Ubuntu 24.04 (Noble) が必要です (現在: ${PRETTY_NAME})" >&2
    echo "先に 01_upgrade_to_noble.sh を実行してください。" >&2
    exit 1
fi

echo "=== ロケール設定 (UTF-8) ==="
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== universe リポジトリ有効化 ==="
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

echo "=== ROS 2 apt リポジトリ追加 ==="
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb

echo "=== ROS 2 ${ROS_DISTRO} インストール ==="
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-dev-tools "ros-${ROS_DISTRO}-desktop"

echo "=== rosdep 初期化 ==="
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

echo "=== ~/.bashrc 設定 ==="
SETUP_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -qxF "${SETUP_LINE}" ~/.bashrc; then
    echo "${SETUP_LINE}" >> ~/.bashrc
fi

echo "完了: 新しいターミナルを開くか 'source ~/.bashrc' を実行してください。"
