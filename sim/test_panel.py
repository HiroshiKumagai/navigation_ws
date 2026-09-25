"""テストパネル: シミュレーションで異常を再現するための操作画面（製品の UI とは別）。

sim_test_hooks.py の試験用インタフェースを ROS 2 で直接操作する。ミッションサーバは経由しない。
    - 安全系: 模擬スイッチ（解除 / 保護停止 / 非常停止）と非常停止のリセット
    - 機器異常: LiDAR（左前 / 右後）、台車ドライバの停止
    - ワープ: 地図上で指定した姿勢へロボットを瞬間移動（オドメトリは追従しないので自己位置喪失になる）
地図には、シミュレーションの真値（青）と AMCL の推定（オレンジ）を重ねて表示する。

使い方: .venv/bin/python sim/test_panel.py [--map <map.yaml>]
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PIL import Image
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QGraphicsScene,
    QGraphicsView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.ndimage import distance_transform_edt
from std_msgs.msg import Empty, String, UInt8
from std_srvs.srv import SetBool

WS_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MAP = WS_DIR / "src" / "o3dyn_navigation" / "maps" / "warehouse.yaml"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# 車体外形（base_link 基準）。src/o3dyn_navigation/config/nav2_params.yaml の footprint と同じ
FOOTPRINT = [(0.98, 0.74), (0.98, -0.74), (-1.09, -0.74), (-1.09, 0.74)]
WARP_CLEARANCE = 1.4  # [m] ランダムワープで障害物から離す距離（外接円半径 約 1.26 m + 余裕）

SAFETY_LABELS = {0: ("正常", "#2e7d32"), 1: ("保護停止", "#ef6c00"), 2: ("非常停止", "#c62828")}
DEVICES = [("lidar_front_left", "LiDAR 左前"), ("lidar_rear_right", "LiDAR 右後"), ("drive", "台車ドライバ")]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosClient(Node):
    """テストパネルの ROS 2 通信"""

    def __init__(self):
        super().__init__("test_panel")
        self.safety_state = None
        self.status = None
        self.truth = None  # (x, y, yaw)
        self.amcl = None
        self.create_subscription(UInt8, "/safety/state", self._on_safety, LATCHED)
        self.create_subscription(String, "/sim/status", self._on_status, LATCHED)
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, LATCHED)
        self.switch_pub = self.create_publisher(UInt8, "/sim/safety_switch", 10)
        self.reset_pub = self.create_publisher(Empty, "/safety/reset", 10)
        self.warp_pub = self.create_publisher(PoseStamped, "/sim/warp", 10)
        self.fault_clients = {d: self.create_client(SetBool, f"/sim/fault/{d}") for d, _ in DEVICES}

    def _on_safety(self, msg):
        self.safety_state = msg.data

    def _on_status(self, msg):
        self.status = json.loads(msg.data)

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_amcl(self, msg):
        p = msg.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def set_safety_switch(self, value):
        self.switch_pub.publish(UInt8(data=value))

    def reset_safety(self):
        self.reset_pub.publish(Empty())

    def set_fault(self, device, on):
        client = self.fault_clients[device]
        if not client.service_is_ready():
            self.get_logger().warn(f"/sim/fault/{device} が見つかりません（シミュレーションは起動中？）")
            return False
        client.call_async(SetBool.Request(data=on))
        return True

    def warp(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.warp_pub.publish(msg)


class OccupancyMap:
    """map_server 形式（PGM + YAML）の地図"""

    def __init__(self, yaml_path):
        info = yaml.safe_load(Path(yaml_path).read_text())
        image_path = Path(yaml_path).parent / info["image"]
        self.image = np.ascontiguousarray(np.array(Image.open(image_path).convert("L")))
        self.height, self.width = self.image.shape
        self.resolution = float(info["resolution"])
        self.origin = (float(info["origin"][0]), float(info["origin"][1]))
        free = self.image >= 250
        # 各セルから最寄りの「空きでないセル」までの距離 [m]
        self.clearance = distance_transform_edt(free) * self.resolution

    def to_pixel(self, x, y):
        return (x - self.origin[0]) / self.resolution, self.height - (y - self.origin[1]) / self.resolution

    def to_world(self, px, py):
        return self.origin[0] + px * self.resolution, self.origin[1] + (self.height - py) * self.resolution

    def random_free_pose(self, clearance):
        rows, cols = np.nonzero(self.clearance >= clearance)
        if len(rows) == 0:
            return None
        i = random.randrange(len(rows))
        x, y = self.to_world(cols[i] + 0.5, rows[i] + 0.5)
        return x, y, random.uniform(-math.pi, math.pi)


class MapView(QGraphicsView):
    """地図の表示と、ワープ先の指定（クリックで位置、ドラッグで向き）"""

    def __init__(self, occupancy_map, on_target_changed):
        super().__init__()
        self.map = occupancy_map
        self.on_target_changed = on_target_changed
        self.target = None
        self._press_pos = None

        self.setScene(QGraphicsScene(self))
        self.setRenderHint(QPainter.Antialiasing)
        self.setMinimumSize(420, 560)
        image = QImage(self.map.image.data, self.map.width, self.map.height, self.map.width, QImage.Format_Grayscale8)
        self.scene().addPixmap(QPixmap.fromImage(image.copy()))
        self.truth_item = self._robot_item(QColor(30, 100, 220, 170))
        self.amcl_item = self._robot_item(QColor(255, 140, 0, 110))
        self.target_item = self._robot_item(QColor(200, 0, 160, 90), dashed=True)
        for item in (self.truth_item, self.amcl_item, self.target_item):
            item.setVisible(False)

    def _robot_item(self, color, dashed=False):
        pen = QPen(color.darker(150), 0)
        if dashed:
            pen.setStyle(Qt.DashLine)
        item = self.scene().addPolygon(QPolygonF(), pen, QBrush(color))
        item.setZValue(10)
        return item

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.scene().itemsBoundingRect(), Qt.KeepAspectRatio)

    def _polygon(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        points = [QPointF(*self.map.to_pixel(x + c * fx - s * fy, y + s * fx + c * fy)) for fx, fy in FOOTPRINT]
        # 前方を示す三角形
        nose = [(0.98, 0.3), (1.5, 0.0), (0.98, -0.3)]
        points += [QPointF(*self.map.to_pixel(x + c * fx - s * fy, y + s * fx + c * fy)) for fx, fy in nose]
        points.append(points[0])
        return QPolygonF(points)

    def show_pose(self, item, pose):
        item.setVisible(pose is not None)
        if pose is not None:
            item.setPolygon(self._polygon(*pose))

    def set_target(self, pose):
        self.target = pose
        self.show_pose(self.target_item, pose)
        self.on_target_changed(pose)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            p = self.mapToScene(event.position().toPoint())
            self._press_pos = self.map.to_world(p.x(), p.y())
            yaw = self.target[2] if self.target else 0.0
            self.set_target((*self._press_pos, yaw))

    def mouseMoveEvent(self, event):
        if self._press_pos is not None:
            p = self.mapToScene(event.position().toPoint())
            x, y = self.map.to_world(p.x(), p.y())
            if math.hypot(x - self._press_pos[0], y - self._press_pos[1]) > 0.3:
                self.set_target((*self._press_pos, math.atan2(y - self._press_pos[1], x - self._press_pos[0])))

    def mouseReleaseEvent(self, event):
        self._press_pos = None


class TestPanel(QMainWindow):
    def __init__(self, ros, occupancy_map):
        super().__init__()
        self.ros = ros
        self.map = occupancy_map
        self.setWindowTitle("テストパネル（シミュレーション異常の再現）")

        root = QWidget()
        layout = QHBoxLayout(root)
        side = QVBoxLayout()
        layout.addLayout(side)
        self.map_view = MapView(occupancy_map, self._on_target_changed)
        layout.addWidget(self.map_view, stretch=1)
        self.setCentralWidget(root)

        # 状態
        status_box = QGroupBox("状態")
        status_layout = QGridLayout(status_box)
        self.safety_label = QLabel("—")
        self.safety_label.setAlignment(Qt.AlignCenter)
        self.safety_label.setStyleSheet("font-size: 18px; font-weight: bold; padding: 6px;")
        status_layout.addWidget(QLabel("安全系"), 0, 0)
        status_layout.addWidget(self.safety_label, 0, 1)
        self.truth_label = QLabel("—")
        self.amcl_label = QLabel("—")
        self.error_label = QLabel("—")
        status_layout.addWidget(QLabel("真値（青）"), 1, 0)
        status_layout.addWidget(self.truth_label, 1, 1)
        status_layout.addWidget(QLabel("AMCL（橙）"), 2, 0)
        status_layout.addWidget(self.amcl_label, 2, 1)
        status_layout.addWidget(QLabel("推定誤差"), 3, 0)
        status_layout.addWidget(self.error_label, 3, 1)
        side.addWidget(status_box)

        # 安全系
        safety_box = QGroupBox("安全系（模擬スイッチ）")
        safety_layout = QVBoxLayout(safety_box)
        self.switch_group = QButtonGroup(self)
        for value, label in [(0, "解除"), (1, "保護停止"), (2, "非常停止")]:
            button = QRadioButton(label)
            self.switch_group.addButton(button, value)
            safety_layout.addWidget(button)
        self.switch_group.button(0).setChecked(True)
        self.switch_group.idClicked.connect(self.ros.set_safety_switch)
        reset_button = QPushButton("非常停止リセット")
        reset_button.setToolTip("スイッチが「解除」のときだけ有効（本来はミッションサーバの ResetSafetyStop から）")
        reset_button.clicked.connect(self.ros.reset_safety)
        safety_layout.addWidget(reset_button)
        side.addWidget(safety_box)

        # 機器異常
        fault_box = QGroupBox("機器異常（ON で停止）")
        fault_layout = QVBoxLayout(fault_box)
        self.fault_buttons = {}
        for device, label in DEVICES:
            button = QPushButton(label)
            button.setCheckable(True)
            button.toggled.connect(lambda on, d=device: self._on_fault_toggled(d, on))
            fault_layout.addWidget(button)
            self.fault_buttons[device] = button
        side.addWidget(fault_box)

        # ワープ
        warp_box = QGroupBox("ワープ（地図をクリック / ドラッグで向き）")
        warp_layout = QVBoxLayout(warp_box)
        self.target_label = QLabel("行き先: 未指定")
        warp_layout.addWidget(self.target_label)
        buttons = QHBoxLayout()
        random_button = QPushButton("ランダム")
        random_button.clicked.connect(self._on_random_target)
        self.warp_button = QPushButton("ワープ実行")
        self.warp_button.setEnabled(False)
        self.warp_button.clicked.connect(self._on_warp)
        buttons.addWidget(random_button)
        buttons.addWidget(self.warp_button)
        warp_layout.addLayout(buttons)
        side.addWidget(warp_box)
        side.addStretch()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    # ---- 操作 ----
    def _on_fault_toggled(self, device, on):
        button = self.fault_buttons[device]
        button.setStyleSheet("background-color: #c62828; color: white;" if on else "")
        if not self.ros.set_fault(device, on):
            button.blockSignals(True)
            button.setChecked(not on)
            button.setStyleSheet("")
            button.blockSignals(False)

    def _on_target_changed(self, pose):
        if pose is None:
            self.target_label.setText("行き先: 未指定")
            self.warp_button.setEnabled(False)
        else:
            x, y, yaw = pose
            clearance = self._clearance_at(x, y)
            note = "" if clearance >= WARP_CLEARANCE else "  ※障害物に近い"
            self.target_label.setText(f"行き先: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°){note}")
            self.warp_button.setEnabled(True)

    def _clearance_at(self, x, y):
        px, py = self.map.to_pixel(x, y)
        col, row = int(px), int(py)
        if 0 <= row < self.map.height and 0 <= col < self.map.width:
            return float(self.map.clearance[row, col])
        return 0.0

    def _on_random_target(self):
        pose = self.map.random_free_pose(WARP_CLEARANCE)
        if pose is not None:
            self.map_view.set_target(pose)

    def _on_warp(self):
        if self.map_view.target is not None:
            self.ros.warp(*self.map_view.target)

    # ---- 定期更新 ----
    def _tick(self):
        rclpy.spin_once(self.ros, timeout_sec=0.0)
        for _ in range(20):
            rclpy.spin_once(self.ros, timeout_sec=0.0)

        state = self.ros.safety_state
        if state is None:
            self.safety_label.setText("未接続")
            self.safety_label.setStyleSheet("font-size: 18px; font-weight: bold; padding: 6px; color: gray;")
        else:
            text, color = SAFETY_LABELS.get(state, (str(state), "gray"))
            status = self.ros.status or {}
            if state == 2 and status.get("safety_switch") != 2:
                text += "（リセット待ち）"
            self.safety_label.setText(text)
            self.safety_label.setStyleSheet(
                f"font-size: 18px; font-weight: bold; padding: 6px; color: white; background-color: {color};"
            )

        # シミュレーション側の状態をボタンに反映（別のパネルやコマンドから変えた場合も追従）
        if self.ros.status:
            for device, button in self.fault_buttons.items():
                on = bool(self.ros.status["faults"].get(device, False))
                if button.isChecked() != on:
                    button.blockSignals(True)
                    button.setChecked(on)
                    button.setStyleSheet("background-color: #c62828; color: white;" if on else "")
                    button.blockSignals(False)
            switch = self.ros.status.get("safety_switch", 0)
            if self.switch_group.checkedId() != switch:
                self.switch_group.button(switch).setChecked(True)

        def fmt(pose):
            return "—" if pose is None else f"({pose[0]:6.2f}, {pose[1]:6.2f}, {math.degrees(pose[2]):5.0f}°)"

        self.truth_label.setText(fmt(self.ros.truth))
        self.amcl_label.setText(fmt(self.ros.amcl))
        if self.ros.truth and self.ros.amcl:
            dx, dy = self.ros.amcl[0] - self.ros.truth[0], self.ros.amcl[1] - self.ros.truth[1]
            dyaw = math.degrees(math.atan2(math.sin(self.ros.amcl[2] - self.ros.truth[2]), math.cos(self.ros.amcl[2] - self.ros.truth[2])))
            self.error_label.setText(f"{math.hypot(dx, dy):.2f} m / {dyaw:+.0f}°")
        self.map_view.show_pose(self.map_view.truth_item, self.ros.truth)
        self.map_view.show_pose(self.map_view.amcl_item, self.ros.amcl)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default=str(DEFAULT_MAP), help="map_server 形式の地図 YAML")
    args, qt_args = parser.parse_known_args()

    rclpy.init()
    app = QApplication([sys.argv[0], *qt_args])
    panel = TestPanel(RosClient(), OccupancyMap(args.map))
    panel.resize(1100, 760)
    panel.show()
    try:
        app.exec()
    finally:
        panel.ros.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
