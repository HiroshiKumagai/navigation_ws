#!/usr/bin/env bash
# Ubuntu 22.04 (Jammy) -> 24.04 (Noble) へのアップグレード
# 注意: 事前に重要データのバックアップを取ってください。
#       途中で対話的な確認が入るため、ターミナルから直接実行してください。
set -euo pipefail

. /etc/os-release
if [[ "${UBUNTU_CODENAME:-}" == "noble" ]]; then
    echo "既に Ubuntu 24.04 (Noble) です。02_install_ros2_jazzy.sh を実行してください。"
    exit 0
fi
if [[ "${UBUNTU_CODENAME:-}" != "jammy" ]]; then
    echo "このスクリプトは Ubuntu 22.04 (Jammy) 専用です (現在: ${PRETTY_NAME})" >&2
    exit 1
fi

echo "=== 現在のシステムを最新化 ==="
sudo apt update
sudo apt full-upgrade -y
sudo apt autoremove -y
sudo apt install -y update-manager-core

# LTS間のアップグレードを許可
sudo sed -i 's/^Prompt=.*/Prompt=lts/' /etc/update-manager/release-upgrades

if [[ -f /var/run/reboot-required ]]; then
    echo "パッケージ更新のため再起動が必要です。再起動後にこのスクリプトを再実行してください。"
    exit 0
fi

echo "=== Ubuntu 24.04 へアップグレード開始 ==="
sudo do-release-upgrade

echo "アップグレード完了後は再起動し、02_install_ros2_jazzy.sh を実行してください。"
