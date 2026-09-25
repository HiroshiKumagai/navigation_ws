"""/cmd_vel で前進・横移動・旋回を順に指令し、/odom で全方向移動できているかを確かめる。

使い方: python3 sim/tools/check_omni_motion.py   （Isaac Sim 側は o3dyn_bringup.py を Play 中にしておく）
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

SPEED = 0.5  # [m/s] または [rad/s]
DURATION = 3.0  # [s]（シミュレーション時間）
STEPS = [
    ("前進 (vx)", (SPEED, 0.0, 0.0)),
    ("左へ横移動 (vy)", (0.0, SPEED, 0.0)),
    ("左旋回 (wz)", (0.0, 0.0, SPEED)),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Checker(Node):
    def __init__(self):
        super().__init__("check_omni_motion", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom = None
        self.create_subscription(Odometry, "odom", self._on_odom, 10)

    def _on_odom(self, msg):
        self.odom = msg

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def drive(self, vx, vy, wz, duration):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < duration * 1e9:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = Checker()
    while node.odom is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.drive(0.0, 0.0, 0.0, 1.0)

    for label, (vx, vy, wz) in STEPS:
        x0, y0, th0 = node.pose()
        node.drive(vx, vy, wz, DURATION)
        node.drive(0.0, 0.0, 0.0, 1.0)
        x1, y1, th1 = node.pose()
        # 開始時の姿勢から見た移動量（ロボット座標系）
        dx, dy = x1 - x0, y1 - y0
        fwd = math.cos(th0) * dx + math.sin(th0) * dy
        left = -math.sin(th0) * dx + math.cos(th0) * dy
        dth = math.degrees(math.atan2(math.sin(th1 - th0), math.cos(th1 - th0)))
        expected = SPEED * DURATION
        print(f"{label:14s} 前方 {fwd:+.2f} m, 左方 {left:+.2f} m, 旋回 {dth:+.1f} deg   (指令どおりなら約 {expected:.1f} m / {math.degrees(expected):.0f} deg)")

    node.drive(0.0, 0.0, 0.0, 0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
