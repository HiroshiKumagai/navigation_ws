"""複数の 2D LiDAR のスキャンを、車体中心の 1 本の LaserScan にまとめるノード。

- 各スキャンの点を静的 TF で base_link 座標へ変換する
- 車体外形（footprint）に余裕を足した範囲内の点は、車体そのものを見た点として捨てる
- 出力フレーム（既定 base_scan）から見た方位ごとに最も近い点を残し、360° の LaserScan として配信する

AMCL はスキャンを 1 本しか受け付けないため、角に付けた 2 台の LiDAR をこのノードで 1 本にする。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanMerger(Node):
    def __init__(self):
        super().__init__("scan_merger")
        self.input_topics = self.declare_parameter("input_topics", ["scan_front_left", "scan_rear_right"]).value
        self.output_frame = self.declare_parameter("output_frame", "base_scan").value
        # footprint の矩形 [x_min, x_max, y_min, y_max]（base_link 基準）と、車体とみなす余裕
        self.body_box = self.declare_parameter("body_box", [-1.09, 0.98, -0.74, 0.74]).value
        self.body_margin = self.declare_parameter("body_margin", 0.08).value
        self.angle_increment = math.radians(self.declare_parameter("angle_increment_deg", 0.25).value)
        self.range_min = self.declare_parameter("range_min", 0.05).value
        self.range_max = self.declare_parameter("range_max", 40.0).value

        self.num_bins = int(round(2.0 * math.pi / self.angle_increment))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sensor_poses = {}  # frame_id -> (x, y, yaw) in output_frame
        self.latest_points = {}  # topic -> (stamp, Nx2 points in output_frame)
        self.output_offset = None  # output_frame の base_link 内での (x, y, yaw)

        # 配信は reliable（best effort / reliable どちらの購読側とも接続できる）
        self.publisher = self.create_publisher(LaserScan, "scan", 10)
        for topic in self.input_topics:
            self.create_subscription(
                LaserScan, topic, lambda msg, t=topic: self._on_scan(t, msg), qos_profile_sensor_data
            )

    def _lookup_pose(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time())
        return tf.transform.translation.x, tf.transform.translation.y, _yaw_of(tf.transform.rotation)

    def _on_scan(self, topic, msg):
        try:
            if msg.header.frame_id not in self.sensor_poses:
                self.sensor_poses[msg.header.frame_id] = self._lookup_pose("base_link", msg.header.frame_id)
            if self.output_offset is None:
                self.output_offset = self._lookup_pose("base_link", self.output_frame)
        except TransformException as e:
            self.get_logger().warn(f"TF 待ち: {e}", throttle_duration_sec=5.0)
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= max(msg.range_min, self.range_min)) & (ranges <= msg.range_max)
        ranges, angles = ranges[valid], angles[valid]

        # センサ座標 -> base_link
        sx, sy, syaw = self.sensor_poses[msg.header.frame_id]
        x = sx + ranges * np.cos(angles + syaw)
        y = sy + ranges * np.sin(angles + syaw)

        # 車体の範囲内の点を捨てる
        x_min, x_max, y_min, y_max = self.body_box
        m = self.body_margin
        inside = (x > x_min - m) & (x < x_max + m) & (y > y_min - m) & (y < y_max + m)
        x, y = x[~inside], y[~inside]

        # base_link -> 出力フレーム
        ox, oy, oyaw = self.output_offset
        dx, dy = x - ox, y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        self.latest_points[topic] = (msg.header.stamp, np.column_stack((c * dx - s * dy, s * dx + c * dy)))
        self._publish()

    def _publish(self):
        if not self.latest_points:
            return
        stamp = max((st for st, _ in self.latest_points.values()), key=lambda st: (st.sec, st.nanosec))
        points = np.vstack([p for _, p in self.latest_points.values()])
        ranges = np.full(self.num_bins, np.inf)
        if len(points):
            r = np.hypot(points[:, 0], points[:, 1])
            a = np.arctan2(points[:, 1], points[:, 0])
            idx = ((a + math.pi) / self.angle_increment).astype(int) % self.num_bins
            keep = (r >= self.range_min) & (r <= self.range_max)
            np.minimum.at(ranges, idx[keep], r[keep])

        out = LaserScan()
        out.header.stamp = stamp
        out.header.frame_id = self.output_frame
        out.angle_min = -math.pi
        out.angle_increment = self.angle_increment
        out.angle_max = -math.pi + (self.num_bins - 1) * self.angle_increment
        out.range_min = self.range_min
        out.range_max = self.range_max
        out.ranges = ranges.astype(np.float32).tolist()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
