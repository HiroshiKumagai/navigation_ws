"""Nav2 に NavigateToPose の目標を送り、結果と到達誤差を表示する。

使い方: python3 sim/tools/send_goal.py X Y [YAW_DEG]
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    nav = BasicNavigator()
    # waitUntilNav2Active() は AMCL の初期位置を自分で設定しようとして待ち続けるため、
    # 初期位置は nav2_params.yaml の set_initial_pose に任せ、ノードの起動だけを待つ
    nav._waitForNodeToActivate("amcl")
    nav._waitForNodeToActivate("bt_navigator")

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = x, y
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    nav.goToPose(goal)

    last_print = None
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            t = fb.navigation_time.sec
            if t != last_print and t % 5 == 0:
                print(f"  {t:3d} s  残り {fb.distance_remaining:5.2f} m  リカバリ {fb.number_of_recoveries} 回", flush=True)
                last_print = t

    result = nav.getResult()
    print("結果:", {TaskResult.SUCCEEDED: "成功", TaskResult.CANCELED: "中断", TaskResult.FAILED: "失敗"}.get(result, result))
    if result == TaskResult.FAILED:
        code, msg = nav.getTaskError() if hasattr(nav, "getTaskError") else (None, None)
        print("エラー:", code, msg)
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
