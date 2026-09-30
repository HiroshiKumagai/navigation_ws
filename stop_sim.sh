#!/usr/bin/env bash
# シミュレーション環境をまとめて終了する（start_sim.sh で起動したもの、個別に起動したものの両方）。
# UI -> テストパネル -> ミッションサーバ -> Nav2 -> Isaac Sim の順に止め、残ったものは強制終了する。
#
# 使い方: ./stop_sim.sh
set -uo pipefail

# 名前と、プロセスのコマンドラインに一致させるパターン（pgrep -f の拡張正規表現）
TARGETS=(
    "UI|python[0-9.]* -m locomotion_ui"
    "テストパネル|sim/test_panel\.py"
    "ミッションサーバ|python[0-9.]* -m mission_server( |$)"
    "起動スクリプト|start_sim\.sh"
    "Nav2 起動|ros2 launch o3dyn_navigation"
    "Nav2 ノード|/opt/ros/jazzy/lib/(nav2_|opennav_docking|rviz2/|joy/joy_node|teleop_twist_joy/|tf2_ros/static_transform_publisher)"
    "o3dyn_navigation ノード|lib/o3dyn_navigation/(scan_merger|cmd_vel_switch)"
    "Isaac Sim|sim/o3dyn_bringup\.py"
)

running() {
    pgrep -f "$1" | grep -v -x "$$" || true
}

stop_target() {
    local name="$1" pattern="$2" pids
    pids=$(running "$pattern")
    [[ -z "$pids" ]] && return
    echo "停止: $name ($(echo $pids | wc -w) プロセス)"
    kill -INT $pids 2>/dev/null || true
    for _ in $(seq 1 20); do
        [[ -z "$(running "$pattern")" ]] && return
        sleep 0.5
    done
    pids=$(running "$pattern")
    [[ -z "$pids" ]] && return
    kill -TERM $pids 2>/dev/null || true
    sleep 2
    pids=$(running "$pattern")
    if [[ -n "$pids" ]]; then
        echo "  強制終了: $name"
        kill -KILL $pids 2>/dev/null || true
    fi
}

for entry in "${TARGETS[@]}"; do
    stop_target "${entry%%|*}" "${entry#*|}"
done

left=0
for entry in "${TARGETS[@]}"; do
    if [[ -n "$(running "${entry#*|}")" ]]; then
        echo "残っています: ${entry%%|*}" >&2
        left=1
    fi
done
[[ $left -eq 0 ]] && echo "すべて停止しました"
exit $left
