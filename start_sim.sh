#!/usr/bin/env bash
# シミュレーション環境（Isaac Sim + Nav2 + ゲームパッド）を一括で起動する。
# Ctrl+C で Nav2 と Isaac Sim の両方を終了する。
#
# 使い方: ./start_sim.sh [--headless] [--no-nav] [--no-rviz] [--no-joy] [--mode navigation|joystick] [--panel] [--server]
#   --headless   Isaac Sim を画面なしで起動する
#   --no-nav     Isaac Sim だけを起動する（Nav2 は起動しない）
#   --no-rviz    rviz2 を起動しない
#   --no-joy     ゲームパッド（joy_node / teleop_twist_joy）を起動しない
#   --mode       指令切替ノードの初期モード（既定 navigation）
#   --panel      テストパネル（異常の再現用）も起動する
#   --server     ミッションサーバ（gRPC :50051）も起動する
#
# ROS_DOMAIN_ID は既定で 42（同じネットワーク上の他の ROS 2 ノードと混ざらないように）。
# 環境変数で上書きできる: ROS_DOMAIN_ID=10 ./start_sim.sh
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

HEADLESS=""
NAV=true
RVIZ=true
JOY=true
MODE=navigation
PANEL=false
SERVER=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) HEADLESS="--headless" ;;
        --no-nav) NAV=false ;;
        --no-rviz) RVIZ=false ;;
        --no-joy) JOY=false ;;
        --mode) MODE="$2"; shift ;;
        --panel) PANEL=true ;;
        --server) SERVER=true ;;
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "不明なオプション: $1" >&2; exit 1 ;;
    esac
    shift
done

LOG_DIR="$WS_DIR/log/sim"
mkdir -p "$LOG_DIR"
SIM_LOG="$LOG_DIR/isaacsim_$(date +%Y%m%d_%H%M%S).log"

# ROS 2 環境（set -u のまま source すると未定義変数で止まるため一時的に解除）
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
    echo "=== ワークスペースをビルドします ==="
    (cd "$WS_DIR" && colcon build --symlink-install)
fi
source "$WS_DIR/install/setup.bash"
set -u

SIM_PID=""
PANEL_PID=""
SERVER_PID=""
cleanup() {
    if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
        kill "$SERVER_PID" 2>/dev/null || true
    fi
    if [[ -n "$PANEL_PID" ]] && kill -0 "$PANEL_PID" 2>/dev/null; then
        kill "$PANEL_PID" 2>/dev/null || true
    fi
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        echo "=== Isaac Sim を終了します ==="
        # python.sh の子プロセス（Isaac Sim 本体）ごと止めるため、プロセスグループに送る
        kill -TERM -- "-$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "=== Isaac Sim を起動します（ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ログ: $SIM_LOG）==="
setsid "$ISAACSIM_PATH/python.sh" "$WS_DIR/sim/o3dyn_bringup.py" $HEADLESS > "$SIM_LOG" 2>&1 &
SIM_PID=$!

# 準備完了（O3DYN_BRINGUP_READY）まで待つ。初回はシェーダのコンパイルで数分かかる
for _ in $(seq 1 600); do
    if grep -q "O3DYN_BRINGUP_READY" "$SIM_LOG"; then
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "Isaac Sim が起動途中で終了しました。ログを確認してください: $SIM_LOG" >&2
        exit 1
    fi
    sleep 1
done
if ! grep -q "O3DYN_BRINGUP_READY" "$SIM_LOG"; then
    echo "Isaac Sim の起動待ちがタイムアウトしました: $SIM_LOG" >&2
    exit 1
fi
grep "O3DYN_BRINGUP_READY" "$SIM_LOG"

if [[ "$PANEL" == true ]]; then
    if [[ ! -x "$WS_DIR/.venv/bin/python" ]]; then
        echo "テストパネル用の Python 環境（.venv）がありません。README の手順で作成してください。" >&2
    else
        echo "=== テストパネルを起動します ==="
        "$WS_DIR/.venv/bin/python" "$WS_DIR/sim/test_panel.py" > "$LOG_DIR/test_panel.log" 2>&1 &
        PANEL_PID=$!
    fi
fi

if [[ "$SERVER" == true ]]; then
    echo "=== ミッションサーバを起動します（ログ: $LOG_DIR/mission_server.log）==="
    "$WS_DIR/.venv/bin/python" -m mission_server > "$LOG_DIR/mission_server.log" 2>&1 &
    SERVER_PID=$!
fi

if [[ "$NAV" == true ]]; then
    echo "=== Nav2 を起動します ==="
    ros2 launch o3dyn_navigation navigation.launch.py rviz:="$RVIZ" joy:="$JOY" mode:="$MODE"
else
    echo "=== Isaac Sim のみ起動中（Ctrl+C で終了）==="
    wait "$SIM_PID"
fi
