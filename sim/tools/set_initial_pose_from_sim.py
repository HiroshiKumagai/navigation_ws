"""シミュレーションの真値（/odom。初期位置がワールド原点のとき map と一致）を AMCL の初期位置として送る。

Nav2 を起動し直したときなど、ロボットが原点以外にいる場合に使う。
使い方: python3 sim/tools/set_initial_pose_from_sim.py
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("set_initial_pose_from_sim", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    odom = []
    node.create_subscription(Odometry, "odom", lambda m: odom.append(m), 10)
    pub = node.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
    while not odom or pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.5)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose = odom[-1].pose.pose
    msg.pose.pose.position.z = 0.0
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05**2
    msg.pose.covariance[35] = 0.05**2
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    p = msg.pose.pose.position
    print(f"initialpose を送信: x={p.x:.2f} y={p.y:.2f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
