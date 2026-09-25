"""ROS 2 アダプタ: 中核（LocomotionCore）の副作用を ROS 2 で実行し、ROS 2 の観測を中核に伝える。

    副作用 -> ROS 2                         観測 ROS 2 -> 中核
    NavigateThroughPoints -> Nav2 アクション   Nav2 のフィードバック・結果 -> on_waypoint_reached / on_navigation_* / on_nav_activity
    CancelNavigation -> Nav2 のキャンセル      /amcl_pose の共分散 -> on_localization
    SetCommandMode -> 指令切替ノード            /safety/state -> on_safety_signal
    SetInitialPose -> /initialpose
    ResetSafetyStop -> /safety/reset
    ActivateMap -> map_server の load_map

中核のロックを持ったまま ROS 2 を呼ばないよう、副作用はキューに入れて ROS 2 のスレッド（executor）で実行する。
"""

import math
import queue
import threading
from dataclasses import dataclass
from functools import partial

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Empty, UInt8
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .core.model import (
    ActivateMap,
    CancelNavigation,
    Fault,
    FaultCategory,
    FinishMapping,
    NavActivity,
    NavigateThroughPoints,
    Pose2D,
    ResetSafetyStop,
    SetCommandMode,
    SetInitialPose,
    StartMapping,
)

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# 自己位置の良否のしきい値（仕様書 4 章「厳しめ／標準／緩め」）。
# 推定のばらつき（標準偏差）が lost を超えたら喪失、ok を下回ったら回復（間はヒステリシス）
LOCALIZATION_THRESHOLDS = {
    #          (ok_xy [m], ok_yaw [rad], lost_xy [m], lost_yaw [rad])
    "STRICT": (0.15, 0.10, 0.30, 0.20),
    "STANDARD": (0.25, 0.15, 0.50, 0.35),
    "LOOSE": (0.40, 0.25, 0.80, 0.50),
}

# 回避動作中 -> 通常に戻す条件: リカバリ後に残り距離がこれだけ減った
RECOVERY_CLEAR_DISTANCE = 0.3  # [m]

# Nav2 のエラーコード -> 異常の分類（仕様書 5 章）。載っていないものは経路閉塞
GOAL_UNREACHABLE_CODES = {204, 206, 304, 306}  # GOAL_OUTSIDE_MAP, GOAL_OCCUPIED
DEVICE_ERROR_CODES = {102, 202, 302}  # TF_ERROR


def classify_nav2_error(error_code, error_msg, now):
    if error_code in GOAL_UNREACHABLE_CODES:
        category = FaultCategory.GOAL_UNREACHABLE
    elif error_code in DEVICE_ERROR_CODES:
        category = FaultCategory.DEVICE_ERROR
    else:
        category = FaultCategory.PATH_BLOCKED
    return Fault(category, detail_code=f"NAV2_{error_code}", detail_message=error_msg, occurred_at=now)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass(frozen=True)
class RobotPose:
    stamp: float
    pose: Pose2D
    linear_x: float
    linear_y: float
    angular: float


@dataclass(frozen=True)
class LocalizationInfo:
    localized: bool
    std_xy: float
    std_yaw: float
    level: str


@dataclass
class _Goal:
    seq: int
    total: int
    handle: object = None
    cancel_requested: bool = False
    reached: int = 0
    recoveries: int = 0
    activity: NavActivity = NavActivity.NORMAL
    recovery_distance: float = 0.0


class RosAdapter:
    def __init__(self, store, *, use_sim_time=True, localization_level="STANDARD", node_name="mission_server"):
        self.store = store
        self.core = None
        self.localization_level = localization_level
        self._effects = queue.SimpleQueue()
        self._goal: _Goal | None = None
        self._localized: bool | None = None
        self._amcl_std = (math.inf, math.inf)
        self._odom_twist = (0.0, 0.0, 0.0)
        self._pose: RobotPose | None = None

        self.node = Node(node_name, parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=use_sim_time)])
        n = self.node
        self._nav_to_pose = ActionClient(n, NavigateToPose, "navigate_to_pose")
        self._nav_through_poses = ActionClient(n, NavigateThroughPoses, "navigate_through_poses")
        self._set_joystick_mode = n.create_client(SetBool, "/cmd_vel_switch/set_joystick_mode")
        self._load_map = n.create_client(LoadMap, "/map_server/load_map")
        self._initial_pose_pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._safety_reset_pub = n.create_publisher(Empty, "/safety/reset", 10)
        n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, LATCHED)
        n.create_subscription(UInt8, "/safety/state", self._on_safety_state, LATCHED)
        n.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, n)
        n.create_timer(0.1, self._update_pose)
        self._effect_trigger = n.create_guard_condition(self._drain_effects)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        self._thread = threading.Thread(target=self._executor.spin, name="ros_adapter", daemon=True)

    # ---- 起動・停止 ----
    def attach(self, core):
        self.core = core

    def start(self):
        self._thread.start()

    def shutdown(self):
        self._executor.shutdown()
        self.node.destroy_node()

    # ---- 参照（gRPC 層から） ----
    def latest_pose(self) -> RobotPose | None:
        return self._pose

    def localization(self) -> LocalizationInfo:
        return LocalizationInfo(bool(self._localized), self._amcl_std[0], self._amcl_std[1], self.localization_level)

    # ---- 副作用（中核から。どのスレッドからでも呼べる） ----
    def handle_effect(self, effect):
        self._effects.put(effect)
        self._effect_trigger.trigger()

    def _drain_effects(self):
        while True:
            try:
                effect = self._effects.get_nowait()
            except queue.Empty:
                return
            try:
                self._dispatch(effect)
            except Exception as e:  # 1 つの失敗で後続の副作用を止めない
                self.node.get_logger().error(f"副作用 {effect} の実行に失敗: {e}")

    def _dispatch(self, effect):
        log = self.node.get_logger()
        if isinstance(effect, NavigateThroughPoints):
            self._send_goal(effect)
        elif isinstance(effect, CancelNavigation):
            self._cancel_goal()
        elif isinstance(effect, SetCommandMode):
            self._call(self._set_joystick_mode, SetBool.Request(data=effect.joystick), "指令切替ノード")
        elif isinstance(effect, SetInitialPose):
            pose = effect.pose or self.store.get_point(self.core.active_map_id, effect.point_id).pose
            self._publish_initial_pose(pose)
        elif isinstance(effect, ResetSafetyStop):
            self._safety_reset_pub.publish(Empty())
        elif isinstance(effect, ActivateMap):
            request = LoadMap.Request(map_url=str(self.store.get_map(effect.map_id).yaml_path))
            self._call(self._load_map, request, "map_server")
        elif isinstance(effect, (StartMapping, FinishMapping)):
            log.warn(f"地図作成は未実装です（ステップ 4）: {effect}")
        else:
            log.error(f"未知の副作用: {effect}")

    def _call(self, client, request, name):
        if not client.service_is_ready():
            self.node.get_logger().error(f"{name} のサービス {client.srv_name} が見つかりません")
            return
        client.call_async(request)

    # ---- Nav2 ----
    def _send_goal(self, effect):
        now = self._now()
        poses = []
        for point_id in effect.point_ids:
            p = self.store.get_point(effect.map_id, point_id).pose
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x, msg.pose.position.y = p.x, p.y
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(p.yaw / 2), math.cos(p.yaw / 2)
            poses.append(msg)

        single = len(poses) == 1
        client = self._nav_to_pose if single else self._nav_through_poses
        if not client.server_is_ready():
            self.core.on_navigation_failed(
                effect.goal_seq, Fault(FaultCategory.DEVICE_ERROR, "NAV2_UNAVAILABLE", "Nav2 に接続できません", now)
            )
            return
        goal = NavigateToPose.Goal(pose=poses[0]) if single else NavigateThroughPoses.Goal(poses=poses)
        self._goal = _Goal(seq=effect.goal_seq, total=len(poses))
        future = client.send_goal_async(goal, feedback_callback=partial(self._on_feedback, effect.goal_seq))
        future.add_done_callback(partial(self._on_goal_response, effect.goal_seq))

    def _cancel_goal(self):
        goal = self._goal
        if goal is None:
            return
        goal.cancel_requested = True
        if goal.handle is not None:
            goal.handle.cancel_goal_async()
        # handle がまだ無い場合は、受理の応答が来た時点でキャンセルする（_on_goal_response）

    def _on_goal_response(self, seq, future):
        handle = future.result()
        goal = self._goal
        if goal is None or goal.seq != seq:
            if handle.accepted:
                handle.cancel_goal_async()  # 既に次の目標に移っている
            return
        if not handle.accepted:
            self.core.on_navigation_failed(
                seq, Fault(FaultCategory.PATH_BLOCKED, "NAV2_REJECTED", "Nav2 が目標を受け付けませんでした", self._now())
            )
            return
        goal.handle = handle
        if goal.cancel_requested:
            handle.cancel_goal_async()
            return
        handle.get_result_async().add_done_callback(partial(self._on_result, seq))

    def _on_feedback(self, seq, feedback_msg):
        goal = self._goal
        if goal is None or goal.seq != seq or goal.cancel_requested:
            return
        fb = feedback_msg.feedback
        if goal.total > 1:
            reached = goal.total - int(fb.number_of_poses_remaining)
            for index in range(goal.reached, max(goal.reached, reached)):
                self.core.on_waypoint_reached(seq, index)
            goal.reached = max(goal.reached, reached)

        # 回避動作中: リカバリ回数が増えたら。残り距離が十分減ったら通常に戻す
        activity = goal.activity
        if fb.number_of_recoveries > goal.recoveries:
            goal.recoveries = int(fb.number_of_recoveries)
            goal.recovery_distance = float(fb.distance_remaining)
            activity = NavActivity.RECOVERING
        elif activity == NavActivity.RECOVERING and fb.distance_remaining < goal.recovery_distance - RECOVERY_CLEAR_DISTANCE:
            activity = NavActivity.NORMAL
        if activity != goal.activity:
            goal.activity = activity
            self.core.on_nav_activity(seq, activity)

    def _on_result(self, seq, future):
        result = future.result()
        goal = self._goal
        if goal is not None and goal.seq == seq:
            self._goal = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.core.on_navigation_succeeded(seq)
        elif result.status == GoalStatus.STATUS_ABORTED:
            r = result.result
            self.core.on_navigation_failed(seq, classify_nav2_error(int(r.error_code), r.error_msg, self._now()))
        # CANCELED はこちらからのキャンセル（中核は既に次の状態に移っている）

    # ---- 観測 ----
    def _on_amcl_pose(self, msg):
        cov = msg.pose.covariance
        std_xy = math.sqrt(max(cov[0], cov[7], 0.0))
        std_yaw = math.sqrt(max(cov[35], 0.0))
        self._amcl_std = (std_xy, std_yaw)
        ok_xy, ok_yaw, lost_xy, lost_yaw = LOCALIZATION_THRESHOLDS[self.localization_level]
        localized = self._localized
        if std_xy > lost_xy or std_yaw > lost_yaw:
            localized = False
        elif std_xy < ok_xy and std_yaw < ok_yaw:
            localized = True
        if localized is not None and localized != self._localized:
            self._localized = localized
            self.core.on_localization(localized)

    def _on_safety_state(self, msg):
        self.core.on_safety_signal(int(msg.data))

    def _on_odom(self, msg):
        t = msg.twist.twist
        self._odom_twist = (t.linear.x, t.linear.y, t.angular.z)

    def _update_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        t = tf.transform
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        self._pose = RobotPose(
            stamp=stamp,
            pose=Pose2D(t.translation.x, t.translation.y, yaw_of(t.rotation)),
            linear_x=self._odom_twist[0],
            linear_y=self._odom_twist[1],
            angular=self._odom_twist[2],
        )

    # ---- 内部 ----
    def _publish_initial_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose.x, pose.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.2**2
        msg.pose.covariance[35] = 0.1**2
        self._initial_pose_pub.publish(msg)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9
