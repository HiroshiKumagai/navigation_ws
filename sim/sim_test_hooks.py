"""異常を再現するための試験用フック（テストパネルから操作する）。o3dyn_bringup.py から使う。

製品側（ミッションサーバ）から見えるのは、安全系の信号と、センサ・台車ドライバの出力の途絶だけにする。

ROS 2 インタフェース:
    /safety/state              配信 std_msgs/UInt8      安全系の状態 0=正常, 1=保護停止, 2=非常停止（ミッションサーバが見る）
    /safety/reset              購読 std_msgs/Empty      非常停止のリセット（スイッチが解除されているときだけ有効）
    /sim/safety_switch         購読 std_msgs/UInt8      模擬スイッチ 0=解除, 1=保護停止, 2=非常停止
    /sim/fault/<device>        サービス std_srvs/SetBool 機器異常の ON/OFF（lidar_front_left, lidar_rear_right, drive）
    /sim/warp                  購読 geometry_msgs/PoseStamped  map（=ワールド）座標の姿勢へ瞬間移動
    /sim/ground_truth          配信 nav_msgs/Odometry   ワールド座標での真値（map -> base_link）
    /sim/status                配信 std_msgs/String     試験用の状態（JSON）

オドメトリ:
    /odom はシミュレーションの真値だが、ワープの前後で連続するように補正して配信する。
    実機で人に押されてもホイールオドメトリが動きを捉えないのと同じ状況にし、自己位置喪失を再現するため。
"""

import json
import math

import omni.graph.core as og
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from isaacsim.core.experimental.prims import Articulation
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String, UInt8
from std_srvs.srv import SetBool

SAFETY_NORMAL, SAFETY_PROTECTIVE, SAFETY_EMERGENCY = 0, 1, 2
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw_from_xyzw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_mul_xyzw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


class SimTestHooks:
    def __init__(self, graph_path, robot_path, lidar_names):
        self.graph_path = graph_path
        self.articulation = Articulation(robot_path)

        # 状態
        self.safety_switch = SAFETY_NORMAL
        self.emergency_latched = False
        self.faults = {f"lidar_{name}": False for name in lidar_names}
        self.faults["drive"] = False
        self.pending_warp = None
        self.rebase_frames = 0
        self.odom_offset = (0.0, 0.0, 0.0)  # 配信する odom = offset ∘ 真値（x, y, yaw）
        self.last_published = (0.0, 0.0, 0.0)

        # OmniGraph の属性
        def attr(path):
            return og.Controller.attribute(f"{graph_path}/{path}")

        self.sim_time = attr("SimTime.outputs:simulationTime")
        self.drive_gate = attr("DriveGate.inputs:condition")
        self.odom_position = attr("ComputeOdom.outputs:position")
        self.odom_orientation = attr("ComputeOdom.outputs:orientation")
        self.pub_odom_position = attr("PubOdom.inputs:position")
        self.pub_odom_orientation = attr("PubOdom.inputs:orientation")
        self.pub_tf_translation = attr("PubOdomTf.inputs:translation")
        self.pub_tf_rotation = attr("PubOdomTf.inputs:rotation")
        self.lidar_enabled = {f"lidar_{name}": attr(f"PubScan_{name}.inputs:enabled") for name in lidar_names}

        # ROS 2（Isaac Sim の ROS 2 ブリッジとは別のコンテキスト）
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("sim_test_hooks", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.safety_pub = self.node.create_publisher(UInt8, "/safety/state", LATCHED)
        self.status_pub = self.node.create_publisher(String, "/sim/status", LATCHED)
        self.truth_pub = self.node.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.node.create_subscription(UInt8, "/sim/safety_switch", self._on_safety_switch, 10)
        self.node.create_subscription(Empty, "/safety/reset", self._on_safety_reset, 10)
        self.node.create_subscription(PoseStamped, "/sim/warp", self._on_warp, 10)
        for device in self.faults:
            self.node.create_service(SetBool, f"/sim/fault/{device}", lambda req, res, d=device: self._on_fault(d, req, res))

        self.frame = 0
        self._publish_state()

    # ---- 状態 ----
    @property
    def safety_state(self):
        if self.emergency_latched:
            return SAFETY_EMERGENCY
        if self.safety_switch == SAFETY_PROTECTIVE:
            return SAFETY_PROTECTIVE
        return SAFETY_NORMAL

    @property
    def drive_enabled(self):
        """台車が速度指令で動ける状態か（安全停止中・台車ドライバ異常中は動かない）"""
        return self.safety_state == SAFETY_NORMAL and not self.faults["drive"]

    def _publish_state(self):
        self.safety_pub.publish(UInt8(data=self.safety_state))
        status = {
            "safety_state": self.safety_state,
            "safety_switch": self.safety_switch,
            "emergency_latched": self.emergency_latched,
            "faults": self.faults,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # ---- コールバック ----
    def _on_safety_switch(self, msg):
        self.safety_switch = msg.data if msg.data in (SAFETY_PROTECTIVE, SAFETY_EMERGENCY) else SAFETY_NORMAL
        if self.safety_switch == SAFETY_EMERGENCY:
            self.emergency_latched = True
        self.node.get_logger().info(f"安全スイッチ: {self.safety_switch} -> 安全系の状態 {self.safety_state}")
        self._publish_state()

    def _on_safety_reset(self, _msg):
        if self.emergency_latched and self.safety_switch != SAFETY_EMERGENCY:
            self.emergency_latched = False
            self.node.get_logger().info("非常停止をリセットしました")
        self._publish_state()

    def _on_fault(self, device, request, response):
        self.faults[device] = bool(request.data)
        if device in self.lidar_enabled:
            self.lidar_enabled[device].set(not request.data)
        if device == "drive":
            # 台車ドライバの停止: オドメトリ・関節状態の配信を止める（速度指令は drive_enabled で無視）
            self.drive_gate.set(not request.data)
        self.node.get_logger().info(f"機器異常 {device}: {'ON' if request.data else 'OFF'}")
        self._publish_state()
        response.success = True
        response.message = json.dumps(self.faults)
        return response

    def _on_warp(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.pending_warp = (p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    # ---- 毎フレーム ----
    def before_update(self):
        """simulation_app.update() の前に呼ぶ"""
        self.executor.spin_once(timeout_sec=0.0)

        if self.pending_warp is not None:
            x, y, yaw = self.pending_warp
            self.pending_warp = None
            positions, _ = self.articulation.get_world_poses()
            z = float(positions.numpy()[0][2])
            self.articulation.set_world_poses(
                positions=[[x, y, z]], orientations=[[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]]
            )
            self.articulation.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])
            self.rebase_frames = 3
            self.node.get_logger().info(f"ワープ: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")

        self._relay_odometry()

    def after_update(self):
        """simulation_app.update() の後に呼ぶ"""
        self.frame += 1
        if self.frame % 2 == 0:
            self._publish_ground_truth()
        if self.frame % 6 == 0:
            self.safety_pub.publish(UInt8(data=self.safety_state))

    def _relay_odometry(self):
        """ComputeOdom の真値に補正を掛けて、PubOdom / PubOdomTf の入力に書き込む"""
        pos = self.odom_position.get()
        quat = list(self.odom_orientation.get())  # OmniGraph の quatd は (x, y, z, w)
        tx, ty, tyaw = float(pos[0]), float(pos[1]), _yaw_from_xyzw(quat)

        if self.rebase_frames > 0:
            # ワープ直後: 配信値が直前の値から連続するように補正を取り直す
            px, py, pyaw = self.last_published
            oyaw = pyaw - tyaw
            c, s = math.cos(oyaw), math.sin(oyaw)
            self.odom_offset = (px - (c * tx - s * ty), py - (s * tx + c * ty), oyaw)
            self.rebase_frames -= 1

        ox, oy, oyaw = self.odom_offset
        c, s = math.cos(oyaw), math.sin(oyaw)
        x, y = ox + c * tx - s * ty, oy + s * tx + c * ty
        q = _quat_mul_xyzw([0.0, 0.0, math.sin(oyaw / 2), math.cos(oyaw / 2)], quat)
        self.last_published = (x, y, tyaw + oyaw)

        self.pub_odom_position.set([x, y, float(pos[2])])
        self.pub_tf_translation.set([x, y, float(pos[2])])
        self.pub_odom_orientation.set(q)
        self.pub_tf_rotation.set(q)

    def _publish_ground_truth(self):
        positions, orientations = self.articulation.get_world_poses()
        p = positions.numpy()[0]
        w, qx, qy, qz = orientations.numpy()[0]  # wxyz
        t = float(self.sim_time.get())
        msg = Odometry()
        msg.header.stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, p)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = float(qx), float(qy)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = float(qz), float(w)
        self.truth_pub.publish(msg)

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

