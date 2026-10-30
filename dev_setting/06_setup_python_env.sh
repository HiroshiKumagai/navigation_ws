#!/usr/bin/env bash
# テストパネル・UI・ミッションサーバ用の Python 環境（.venv）を作る。
#   - Qt（PySide6）の画面表示に必要な libxcb-cursor0 をインストール
#   - ROS 2 のパッケージを使えるよう --system-site-packages で仮想環境を作り、requirements.txt を入れる
#   - proto から gRPC のコードを生成し、mission_server と運用画面（operator_ui）を開発モードで入れる
#   - Claude Code 用に、Isaac Sim 同梱のスキル（~/isaacsim/skills）のうち必要なものを .claude/skills にリンクする
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== Qt の表示に必要なライブラリと Python のパッケージ ==="
# テストパネル・ツールが使う numpy / scipy / PIL / yaml はシステムのものを使う（.venv は --system-site-packages）
sudo apt install -y libxcb-cursor0 python3.12-venv python3-numpy python3-scipy python3-pil python3-yaml

echo "=== Python 仮想環境: $WS_DIR/.venv ==="
if [[ ! -x "$WS_DIR/.venv/bin/python" ]]; then
    python3 -m venv --system-site-packages "$WS_DIR/.venv"
fi
# uv など pip を含まない方法で作った .venv にも対応する
"$WS_DIR/.venv/bin/python" -m pip --version > /dev/null 2>&1 || "$WS_DIR/.venv/bin/python" -m ensurepip --upgrade
"$WS_DIR/.venv/bin/python" -m pip install --upgrade pip
"$WS_DIR/.venv/bin/python" -m pip install -r "$WS_DIR/requirements.txt"

"$WS_DIR/scripts/gen_proto.sh"
"$WS_DIR/.venv/bin/python" -m pip install -e "$WS_DIR/mission_server"
"$WS_DIR/.venv/bin/python" -m pip install -e "$WS_DIR/operator_ui"

# Claude Code 用の Isaac Sim スキル（ROS 2・Nav2・移動ロボット・センサに関係するもの）
ISAACSIM_SKILLS="${ISAACSIM_PATH:-$HOME/isaacsim}/skills"
if [[ -d "$ISAACSIM_SKILLS" ]]; then
    mkdir -p "$WS_DIR/.claude/skills"
    for skill in isaac-sim-orchestrator isaac-sim-workflow isaac-sim-troubleshooting isaac-sim-validator \
        isaac-sim-ros2-bridge isaac-sim-ros-workspaces isaac-sim-robot-navigation occupancy-map navigation-primitives \
        urdf-mjcf-to-usd-conversion usd-articulation physics-simulation isaac-sim-sensor isaac-sim-assets \
        isaac-sim-headless-deployment; do
        ln -sfn "$ISAACSIM_SKILLS/$skill" "$WS_DIR/.claude/skills/$skill"
    done
else
    echo "Isaac Sim のスキルが見つかりません（$ISAACSIM_SKILLS）。Claude Code を使わないなら問題ありません"
fi

"$WS_DIR/.venv/bin/python" -c "import PySide6, rclpy, mission_server, locomotion.v1.locomotion_pb2; print('OK: PySide6', PySide6.__version__)"
