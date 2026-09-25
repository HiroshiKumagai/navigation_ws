#!/usr/bin/env bash
# Ubuntu 24.04 アップグレード後の後処理
#  - 無効化された外部リポジトリを noble 向けに再有効化
#  - ROS 2 apt リポジトリ設定を noble 向けに更新
#  - jammy ビルドのまま残ったパッケージを noble 版へ入れ替え
#  - 残りの更新適用と不要パッケージ・残留設定の掃除
set -euo pipefail

. /etc/os-release
if [[ "${UBUNTU_CODENAME:-}" != "noble" ]]; then
    echo "Ubuntu 24.04 (Noble) 上で実行してください (現在: ${PRETTY_NAME})" >&2
    exit 1
fi

SRC_DIR=/etc/apt/sources.list.d
BACKUP_DIR=/etc/apt/sources.list.d.bak-$(date +%Y%m%d%H%M%S)

echo "=== 外部リポジトリを noble 向けに再設定 ==="
sudo tee ${SRC_DIR}/docker.list > /dev/null <<'EOF'
deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu noble stable
EOF

sudo tee ${SRC_DIR}/github_git-lfs.list > /dev/null <<'EOF'
deb [signed-by=/etc/apt/keyrings/github_git-lfs-archive-keyring.gpg] https://packagecloud.io/github/git-lfs/ubuntu/ noble main
EOF

sudo tee ${SRC_DIR}/librealsense.list > /dev/null <<'EOF'
deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo noble main
EOF

sudo tee ${SRC_DIR}/tailscale.list > /dev/null <<'EOF'
# Tailscale packages for ubuntu noble
deb [signed-by=/usr/share/keyrings/tailscale-archive-keyring.gpg] https://pkgs.tailscale.com/stable/ubuntu noble main
EOF

sudo tee ${SRC_DIR}/nvidia-container-toolkit.list > /dev/null <<'EOF'
deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$(ARCH) /
EOF

# 既に deb822 形式で noble 向けになっているものは有効化のみ
sudo sed -i 's/^Enabled: no$/Enabled: yes/' \
    ${SRC_DIR}/hluk-ubuntu-copyq-jammy.sources \
    ${SRC_DIR}/foxglove-studio.sources

echo "=== 旧設定ファイルを ${BACKUP_DIR} へ退避 ==="
sudo mkdir -p "${BACKUP_DIR}"
sudo mv ${SRC_DIR}/*.distUpgrade "${BACKUP_DIR}/"
# jammy のインストールメディア(cdrom)の無効エントリ
sudo mv ${SRC_DIR}/third-party.sources "${BACKUP_DIR}/"

echo "=== ROS 2 apt リポジトリ設定を noble 向けに更新 ==="
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb

echo "=== 更新適用 ==="
sudo apt update
sudo apt full-upgrade -y

# バージョン番号が jammy 版と同じで入れ替わらないものを noble 版で再インストール
REINSTALL=$(apt list '?obsolete' 2>/dev/null | awk -F/ 'NR>1 {print $1}' \
    | while read -r pkg; do apt-cache policy "$pkg" | grep -q 'http' && echo "$pkg"; done || true)
if [[ -n "${REINSTALL}" ]]; then
    echo "=== noble 版で再インストール: ${REINSTALL//$'\n'/ } ==="
    sudo apt install -y --reinstall ${REINSTALL}
fi

sudo apt autoremove -y

echo "=== 削除済みパッケージの残留設定ファイル ==="
RC_PKGS=$(dpkg -l | awk '/^rc/ {print $2}')
if [[ -n "${RC_PKGS}" ]]; then
    echo "${RC_PKGS}" | column
    read -r -p "上記 $(echo "${RC_PKGS}" | wc -l) 件の残留設定を削除しますか? [y/N] " ans
    if [[ "${ans}" =~ ^[Yy]$ ]]; then
        sudo apt purge -y ${RC_PKGS}
    fi
fi

echo
echo "=== 確認: 配布元リポジトリの無いパッケージ(0件なら問題なし) ==="
apt list '?obsolete' 2>/dev/null | tail -n +2

if [[ -f /var/run/reboot-required ]]; then
    echo "再起動が必要です。再起動後に 02_install_ros2_jazzy.sh を実行してください。"
else
    echo "完了。続けて 02_install_ros2_jazzy.sh を実行してください。"
fi
