"""中核 + ROS 2 アダプタをシミュレーションにつないで、移動・一時停止・安全停止の一連の流れを試す（gRPC なし）。

前提: ./start_sim.sh で Isaac Sim と Nav2 が起動していること（指令切替ノードはナビゲーションモード）。
使い方: .venv/bin/python mission_server/tools/try_with_sim.py
"""

import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8

from mission_server.core.model import Role, RobotState as S, TaskStatus
from mission_server.core.state_machine import LocomotionCore
from mission_server.ros_adapter import RosAdapter
from mission_server.store import Store

OP = Role.OPERATOR
DATA_DIR = Path(__file__).resolve().parents[2] / "data"


def main():
    rclpy.init()
    store = Store(DATA_DIR)
    adapter = RosAdapter(store)
    log = []

    def on_status(status):
        task = status.active_task
        line = f"  [{status.sequence:3d}] {status.state.name:11s}"
        if status.nav_activity:
            line += f" {status.nav_activity.name}"
        if status.safety_stop_kind:
            line += f" {status.safety_stop_kind.name}"
        if status.fault:
            line += f" {status.fault.category.name}({status.fault.detail_code})"
        if task:
            line += f"  task={task.status.name} index={task.current_index}/{len(task.point_ids)}"
        if not log or log[-1] != line:
            log.append(line)
            print(line, flush=True)

    core = LocomotionCore(
        active_map_id=store.active_map_id,
        point_exists=store.point_exists,
        map_exists=store.map_exists,
        on_effect=adapter.handle_effect,
        on_status=on_status,
    )
    adapter.attach(core)
    adapter.start()

    # テスト用: シミュレーションのフック（ミッションサーバ本体は使わない）
    helper = rclpy.create_node("try_with_sim_helper")
    warp_pub = helper.create_publisher(PoseStamped, "/sim/warp", 10)
    switch_pub = helper.create_publisher(UInt8, "/sim/safety_switch", 10)

    def wait_until(condition, timeout, label):
        start = time.time()
        while time.time() - start < timeout:
            if condition():
                return time.time() - start
            time.sleep(0.1)
        raise TimeoutError(f"{label} を待ちましたが {timeout} 秒でタイムアウトしました（状態 {core.state.name}）")

    def moved_since(pose, seconds):
        time.sleep(seconds)
        now = adapter.latest_pose().pose
        return math.hypot(now.x - pose.x, now.y - pose.y)

    try:
        wait_until(lambda: adapter.latest_pose() is not None, 30, "ロボットの位置")
        print("1) ホームへワープして初期位置を設定")
        home = store.get_point(store.active_map_id, "home").pose
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = home.x, home.y
        msg.pose.orientation.w = 1.0
        time.sleep(1.0)
        warp_pub.publish(msg)
        time.sleep(1.0)
        core.set_initial_pose(OP, point_id="home")
        wait_until(lambda: adapter.localization().localized, 20, "自己位置の確定")
        assert core.state == S.IDLE

        print("2) 地点 A へ単発移動")
        task = core.start_task(OP, "try-1", ["A"])
        t = wait_until(lambda: core.state == S.IDLE, 120, "A への到達")
        print(f"   -> {task.status.name}（{t:.0f} 秒）")
        assert task.status == TaskStatus.SUCCEEDED

        print("3) C -> home の巡回。途中で一時停止・再開")
        task = core.start_task(OP, "try-2", ["C", "home"])
        time.sleep(6.0)
        core.pause_task(OP, task.task_id)
        stop_pose = adapter.latest_pose().pose
        print(f"   一時停止後 3 秒間の移動量: {moved_since(stop_pose, 3.0):.2f} m")
        core.resume_task(OP, task.task_id)
        t = wait_until(lambda: core.state == S.IDLE, 180, "巡回の完了")
        print(f"   -> {task.status.name}（再開から {t:.0f} 秒、到達した地点 {task.current_index}/{len(task.point_ids)}）")
        assert task.status == TaskStatus.SUCCEEDED

        print("4) B へ移動中に保護停止 -> 解除 -> 再開")
        task = core.start_task(OP, "try-3", ["B"])
        time.sleep(5.0)
        switch_pub.publish(UInt8(data=1))
        wait_until(lambda: core.state == S.SAFETY_STOP, 5, "安全停止")
        stop_pose = adapter.latest_pose().pose
        print(f"   安全停止後 3 秒間の移動量: {moved_since(stop_pose, 3.0):.2f} m")
        switch_pub.publish(UInt8(data=0))
        wait_until(lambda: core.state == S.PAUSED, 5, "一時停止（解除後）")
        core.resume_task(OP, task.task_id)
        wait_until(lambda: core.state == S.IDLE, 120, "B への到達")
        print(f"   -> {task.status.name}")
        assert task.status == TaskStatus.SUCCEEDED
        print("すべて成功")
    finally:
        switch_pub.publish(UInt8(data=0))
        adapter.shutdown()
        helper.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
