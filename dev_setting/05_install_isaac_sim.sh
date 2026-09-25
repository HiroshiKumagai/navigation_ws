#!/usr/bin/env bash
# Isaac Sim 6.1.0 (ワークステーション / standalone zip 版) インストール
# 参考: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html
set -euo pipefail

ISAACSIM_VERSION=6.1.0
ZIP_NAME=isaac-sim-standalone-${ISAACSIM_VERSION}-linux-x86_64.zip
ZIP_URL=https://downloads.isaacsim.nvidia.com/${ZIP_NAME}
ZIP_MD5=b471383a51f259e0af22541f05c34b0e
DOWNLOAD_DIR=${HOME}/Downloads
INSTALL_DIR=${HOME}/isaacsim
REQUIRED_DRIVER=595.58.03

echo "=== NVIDIA ドライバ確認 ==="
DRIVER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
echo "ドライバ: ${DRIVER}"
if ! dpkg --compare-versions "${DRIVER}" ge "${REQUIRED_DRIVER}"; then
    echo "Isaac Sim ${ISAACSIM_VERSION} にはドライバ ${REQUIRED_DRIVER} 以上が必要です。" >&2
    echo "04_update_nvidia_driver.sh を実行して再起動してください。" >&2
    exit 1
fi

if [[ -x "${INSTALL_DIR}/isaac-sim.sh" ]]; then
    echo "${INSTALL_DIR} に既に Isaac Sim があります。再インストールする場合は削除してから実行してください。" >&2
    exit 1
fi

echo "=== ダウンロード (約 13.3GB, 中断しても再実行で続きから) ==="
mkdir -p "${DOWNLOAD_DIR}"
wget -c -O "${DOWNLOAD_DIR}/${ZIP_NAME}" "${ZIP_URL}"

echo "=== チェックサム確認 ==="
echo "${ZIP_MD5}  ${DOWNLOAD_DIR}/${ZIP_NAME}" | md5sum -c -

echo "=== 展開: ${INSTALL_DIR} ==="
sudo apt install -y unzip
mkdir -p "${INSTALL_DIR}"
unzip -q "${DOWNLOAD_DIR}/${ZIP_NAME}" -d "${INSTALL_DIR}"

echo "=== post_install ==="
cd "${INSTALL_DIR}"
./post_install.sh

echo "=== ROS 2 連携用の追加パッケージ ==="
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-ackermann-msgs

echo "=== ~/.bashrc 設定 ==="
PATH_LINE="export ISAACSIM_PATH=${INSTALL_DIR}"
if ! grep -q "ISAACSIM_PATH" ~/.bashrc; then
    echo "${PATH_LINE}" >> ~/.bashrc
fi

echo
read -r -p "ダウンロードした zip (${DOWNLOAD_DIR}/${ZIP_NAME}) を削除しますか? [y/N] " ans
if [[ "${ans}" =~ ^[Yy]$ ]]; then
    rm -f "${DOWNLOAD_DIR}/${ZIP_NAME}"
fi

echo
echo "完了: 起動は '${INSTALL_DIR}/isaac-sim.sh' (初回はシェーダのコンパイルで数分かかります)"
