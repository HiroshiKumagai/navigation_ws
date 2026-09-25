#!/usr/bin/env bash
# NVIDIA ドライバを 595 系へ更新 (Isaac Sim 6.x は 595.58.03 以上が必要)
# 実行後は再起動が必要です。
set -euo pipefail

DRIVER_BRANCH=595
REQUIRED_VERSION=595.58.03

CURRENT=$(cat /proc/driver/nvidia/version 2>/dev/null | grep -oP 'Kernel Module\s+\K[0-9.]+' || true)
echo "現在のドライバ: ${CURRENT:-不明}"
if [[ -n "${CURRENT}" ]] && dpkg --compare-versions "${CURRENT}" ge "${REQUIRED_VERSION}"; then
    echo "既に ${REQUIRED_VERSION} 以上です。更新は不要です。"
    exit 0
fi

echo "=== nvidia-driver-${DRIVER_BRANCH} をインストール (旧ドライバのメタパッケージは削除) ==="
sudo apt update
# 末尾の '-' はそのパッケージを削除する指定
sudo apt install -y "nvidia-driver-${DRIVER_BRANCH}" nvidia-driver-570- nvidia-driver-580-
sudo apt autoremove -y

echo "=== DKMS モジュールの状態 ==="
dkms status | grep -E "^nvidia" || true

echo
echo "完了: 再起動してから 'nvidia-smi' でバージョンが ${DRIVER_BRANCH}.x になっていることを確認してください。"
echo "その後 05_install_isaac_sim.sh を実行してください。"
