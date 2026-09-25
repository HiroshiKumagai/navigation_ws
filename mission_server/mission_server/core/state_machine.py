"""状態遷移とタスク管理の中核（仕様書 3〜5 章）。

API からの指示（role 付き）と ROS 2 からの観測を受け取り、状態を遷移させて、
ROS 2 アダプタが実行すべき副作用（Effect）を on_effect に渡す。ROS 2 にも gRPC にも依存しない。

- 指示の受理・拒否は ACCEPTED_STATES と各メソッドの条件で判定し、拒否は Rejected の派生例外で返す
- 状態が変わるたびに sequence を増やし、on_status に RobotStatus を渡す
- Nav2 の結果は goal_seq で突き合わせ、キャンセル済みの古い目標の結果は無視する
"""

import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import replace

from .model import (
    ActivateMap,
    CancelNavigation,
    Effect,
    Fault,
    FaultCategory,
    FinishMapping,
    Mode,
    NavActivity,
    NavigateThroughPoints,
    NotFound,
    PermissionDenied,
    Pose2D,
    PreconditionFailed,
    RobotState,
    RobotStatus,
    ResetSafetyStop,
    Role,
    SafetyStopKind,
    SetCommandMode,
    SetInitialPose,
    StartMapping,
    Task,
    TaskEvent,
    TaskSource,
    TaskStatus,
)

S = RobotState

# 各指示を受け付ける状態（仕様書 4 章の遷移表と proto の各 RPC のコメント）
ACCEPTED_STATES = {
    "start_task": {S.IDLE},
    "pause_task": {S.NAVIGATING},
    "resume_task": {S.PAUSED},
    "stop_task": {S.NAVIGATING, S.PAUSED, S.FAULT},
    "retry_task": {S.FAULT},
    "set_mode": {S.NOT_READY, S.IDLE, S.FAULT, S.MANUAL},
    "set_initial_pose": {S.NOT_READY, S.FAULT},
    "reset_safety_stop": {S.SAFETY_STOP},
    "activate_map": {S.NOT_READY, S.IDLE},
    "start_mapping": {S.MANUAL},
    "save_mapping": {S.MAPPING},
    "discard_mapping": {S.MAPPING},
}

OPERATOR_ROLES = {Role.OPERATOR, Role.INTEGRATOR}


class LocomotionCore:
    def __init__(
        self,
        *,
        initial_mode: Mode = Mode.OPERATOR,
        active_map_id: str | None = None,
        point_exists: Callable[[str, str], bool] = lambda map_id, point_id: True,
        map_exists: Callable[[str], bool] = lambda map_id: True,
        on_effect: Callable[[Effect], None] = lambda effect: None,
        on_status: Callable[[RobotStatus], None] = lambda status: None,
        clock: Callable[[], float] = time.time,
        new_task_id: Callable[[], str] = lambda: uuid.uuid4().hex,
    ):
        self._lock = threading.RLock()
        self._point_exists = point_exists
        self._map_exists = map_exists
        self._on_effect = on_effect
        self._on_status = on_status
        self._clock = clock
        self._new_task_id = new_task_id

        self.state = S.NOT_READY
        self.mode = initial_mode
        self.active_map_id: str | None = active_map_id  # 起動直後は地図があっても初期位置の設定待ち
        self.localized = False
        self.nav_activity = NavActivity.NORMAL
        self.fault: Fault | None = None
        self.safety_stop_kind: SafetyStopKind | None = None
        self._state_before_safety_stop: RobotState | None = None
        self._mapping_map_id: str | None = None

        self.task: Task | None = None  # 保持中のタスク
        self.tasks: dict[str, Task] = {}  # 実行記録（task_id -> Task）
        self._task_by_request: dict[str, str] = {}
        self._goal_seq = 0
        self._goal_start_index = 0

        self.sequence = 0
        self.updated_at = self._clock()

    # ================================================================
    # 状態の参照
    # ================================================================

    @property
    def joystick_accepting(self):
        return self.mode == Mode.JOYSTICK and self.state in (S.MANUAL, S.MAPPING)

    def status(self) -> RobotStatus:
        with self._lock:
            return RobotStatus(
                sequence=self.sequence,
                updated_at=self.updated_at,
                state=self.state,
                mode=self.mode,
                nav_activity=self.nav_activity if self.state == S.NAVIGATING else None,
                safety_stop_kind=self.safety_stop_kind if self.state == S.SAFETY_STOP else None,
                fault=self.fault if self.state == S.FAULT else None,
                active_map_id=self.active_map_id,
                localized=self.localized,
                joystick_accepting=self.joystick_accepting,
                active_task=replace(self.task, history=list(self.task.history)) if self.task else None,
            )

    def get_task(self, task_id: str) -> Task:
        with self._lock:
            if task_id not in self.tasks:
                raise NotFound(f"タスク {task_id} はありません")
            return self.tasks[task_id]

    # ================================================================
    # API からの指示
    # ================================================================

    def start_task(self, role: Role, request_id: str, point_ids: list[str], expected_map_id: str = "") -> Task:
        """移動指示。point_ids が 1 件なら単発移動、複数なら巡回"""
        with self._lock:
            # 冪等化: 同じ要求 ID の再送には既存のタスクを返す
            if request_id and request_id in self._task_by_request:
                return self.tasks[self._task_by_request[request_id]]

            source = self._task_source_for(role)
            self._require_state("start_task")
            if not point_ids:
                raise PreconditionFailed("地点が指定されていません", self.state, self.mode)
            if expected_map_id and expected_map_id != self.active_map_id:
                raise PreconditionFailed(
                    f"使用中の地図 {self.active_map_id} が指定 {expected_map_id} と異なります", self.state, self.mode
                )
            if not self.localized:
                raise PreconditionFailed("自己位置が確定していません", self.state, self.mode)
            for point_id in point_ids:
                if not self._point_exists(self.active_map_id, point_id):
                    raise NotFound(f"地点 {point_id} は地図 {self.active_map_id} にありません")

            now = self._clock()
            task = Task(
                task_id=self._new_task_id(),
                request_id=request_id,
                source=source,
                map_id=self.active_map_id,
                point_ids=list(point_ids),
                created_at=now,
            )
            self.tasks[task.task_id] = task
            if request_id:
                self._task_by_request[request_id] = task.task_id
            self.task = task
            self._set_state(S.NAVIGATING)
            self._set_task_status(TaskStatus.RUNNING)
            self._navigate()
            self._commit()
            return task

    def pause_task(self, role: Role, task_id: str) -> Task:
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            task = self._require_task(task_id)
            self._require_state("pause_task")
            self._emit(CancelNavigation())
            self._set_state(S.PAUSED)
            self._set_task_status(TaskStatus.PAUSED)
            self._commit()
            return task

    def resume_task(self, role: Role, task_id: str) -> Task:
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            task = self._require_task(task_id)
            self._require_state("resume_task")
            self._set_state(S.NAVIGATING)
            self._set_task_status(TaskStatus.RUNNING)
            self._navigate()
            self._commit()
            return task

    def stop_task(self, role: Role, task_id: str) -> Task:
        """タスクの破棄。移動中・一時停止からは中断、異常停止からは失敗として終わる"""
        with self._lock:
            task = self._require_task(task_id)
            own_upper_task = role == Role.UPPER_SYSTEM and task.source == TaskSource.UPPER
            if not own_upper_task:
                self._require_role(role, OPERATOR_ROLES)
            self._require_state("stop_task")
            if self.state == S.NAVIGATING:
                self._emit(CancelNavigation())
            result = TaskStatus.FAILED if self.state == S.FAULT else TaskStatus.CANCELED
            self._finish_task(result)
            self._set_state(self._idle_or_not_ready())
            self._commit()
            return task

    def retry_task(self, role: Role, task_id: str) -> Task:
        """異常停止から、保持していたタスクを再送する"""
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            task = self._require_task(task_id)
            self._require_state("retry_task")
            self.fault = None
            self._set_state(S.NAVIGATING)
            self._set_task_status(TaskStatus.RUNNING)
            self._navigate()
            self._commit()
            return task

    def set_mode(self, role: Role, mode: Mode) -> RobotState:
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            self._require_state("set_mode")
            if mode == self.mode:
                return self.state

            if self.state == S.FAULT:
                # 異常停止中のモード切替は、保持していたタスクを破棄する（結果は失敗）
                self._finish_task(TaskStatus.FAILED)

            to_joystick = mode == Mode.JOYSTICK
            if to_joystick:
                self._set_state(S.MANUAL)
            elif self.state in (S.MANUAL, S.FAULT):
                self._set_state(self._idle_or_not_ready())
            # 準備未完了・待機での上位指示 <-> UI 指示の切替は状態を変えない

            if to_joystick != (self.mode == Mode.JOYSTICK):
                self._emit(SetCommandMode(joystick=to_joystick))
            self.mode = mode
            self._commit()
            return self.state

    def set_initial_pose(self, role: Role, pose: Pose2D | None = None, point_id: str | None = None) -> RobotState:
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            self._require_state("set_initial_pose")
            if self.active_map_id is None:
                raise PreconditionFailed("使用する地図が選ばれていません", self.state, self.mode)
            if (pose is None) == (point_id is None):
                raise PreconditionFailed("姿勢か地点 ID のどちらか一方を指定してください", self.state, self.mode)
            if point_id is not None and not self._point_exists(self.active_map_id, point_id):
                raise NotFound(f"地点 {point_id} は地図 {self.active_map_id} にありません")

            if self.state == S.FAULT:
                self._finish_task(TaskStatus.FAILED)
            self._emit(SetInitialPose(pose=pose, point_id=point_id))
            self.localized = True
            self._set_state(S.IDLE)
            self._commit()
            return self.state

    def reset_safety_stop(self, role: Role) -> None:
        """非常停止のリセットを安全系に要求する。状態は安全系の信号が解除されてから変わる"""
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            self._require_state("reset_safety_stop")
            if self.safety_stop_kind != SafetyStopKind.EMERGENCY:
                raise PreconditionFailed("非常停止中ではありません", self.state, self.mode)
            self._emit(ResetSafetyStop())

    def activate_map(self, role: Role, map_id: str) -> RobotState:
        """使用する地図の切替。切替後は準備未完了（初期位置の設定待ち）"""
        with self._lock:
            self._require_role(role, OPERATOR_ROLES)
            self._require_state("activate_map")
            if not self._map_exists(map_id):
                raise NotFound(f"地図 {map_id} はありません")
            self._emit(ActivateMap(map_id=map_id))
            self.active_map_id = map_id
            self.localized = False
            self._set_state(S.NOT_READY)
            self._commit()
            return self.state

    def start_mapping(self, role: Role, map_id: str, rescan: bool) -> None:
        with self._lock:
            self._require_role(role, {Role.INTEGRATOR})
            self._require_state("start_mapping")
            exists = self._map_exists(map_id)
            if rescan and not exists:
                raise NotFound(f"地図 {map_id} はありません")
            if not rescan and exists:
                raise PreconditionFailed(f"地図 {map_id} は既にあります", self.state, self.mode)
            self._emit(StartMapping(map_id=map_id, rescan=rescan))
            self._mapping_map_id = map_id
            self._set_state(S.MAPPING)
            self._commit()

    def save_mapping(self, role: Role) -> str:
        """保存すると新しい地図が使用中になり、SLAM の最終推定位置を初期位置として引き継ぐ"""
        with self._lock:
            self._require_role(role, {Role.INTEGRATOR})
            self._require_state("save_mapping")
            self._emit(FinishMapping(save=True))
            map_id = self._mapping_map_id
            self.active_map_id = map_id
            self.localized = True
            self._mapping_map_id = None
            self._set_state(S.MANUAL)
            self._commit()
            return map_id

    def discard_mapping(self, role: Role) -> None:
        """破棄すると自己位置を喪失した扱いになる"""
        with self._lock:
            self._require_role(role, {Role.INTEGRATOR})
            self._require_state("discard_mapping")
            self._emit(FinishMapping(save=False))
            self._mapping_map_id = None
            self.localized = False
            self._set_state(S.MANUAL)
            self._commit()

    # ================================================================
    # ROS 2 からの観測
    # ================================================================

    def on_waypoint_reached(self, goal_seq: int, index_in_goal: int) -> None:
        """巡回中、goal の index_in_goal 番目の地点に到達した"""
        with self._lock:
            if not self._is_current_goal(goal_seq):
                return
            self.task.current_index = self._goal_start_index + index_in_goal + 1
            self._commit()

    def on_navigation_succeeded(self, goal_seq: int) -> None:
        with self._lock:
            if not self._is_current_goal(goal_seq):
                return
            self.task.current_index = len(self.task.point_ids)
            self._finish_task(TaskStatus.SUCCEEDED)
            self._set_state(S.IDLE)
            self._commit()

    def on_navigation_failed(self, goal_seq: int, fault: Fault) -> None:
        with self._lock:
            if not self._is_current_goal(goal_seq):
                return
            self._enter_fault(fault, cancel=False)

    def on_nav_activity(self, goal_seq: int, activity: NavActivity) -> None:
        """回避動作中（Nav2 のリカバリ実行中）かどうか"""
        with self._lock:
            if not self._is_current_goal(goal_seq) or activity == self.nav_activity:
                return
            self.nav_activity = activity
            self._commit()

    def on_device_fault(self, fault: Fault) -> None:
        """死活監視による機器異常。移動中なら異常停止にする"""
        with self._lock:
            if self.state == S.NAVIGATING:
                self._enter_fault(fault, cancel=True)

    def on_localization(self, localized: bool) -> None:
        """自己位置の良否（AMCL の推定のばらつきをしきい値で判定した結果）"""
        with self._lock:
            if self.state == S.MAPPING or localized == self.localized:
                return
            self.localized = localized
            if not localized:
                if self.state == S.NAVIGATING:
                    self._enter_fault(Fault(FaultCategory.LOCALIZATION_LOST, occurred_at=self._clock()), cancel=True)
                    return
                if self.state == S.IDLE:
                    self._set_state(S.NOT_READY)
            self._commit()

    def on_safety_signal(self, level: int) -> None:
        """安全系の信号（0=正常, 1=保護停止, 2=非常停止）"""
        with self._lock:
            kind = {1: SafetyStopKind.PROTECTIVE, 2: SafetyStopKind.EMERGENCY}.get(level)
            if kind is not None:
                if self.state == S.SAFETY_STOP:
                    if kind == self.safety_stop_kind:
                        return
                else:
                    self._state_before_safety_stop = self.state
                    if self.state == S.NAVIGATING:
                        # 直ちに Nav2 の目標をキャンセルし、タスクは一時停止扱いで保持する
                        self._emit(CancelNavigation())
                        self._set_task_status(TaskStatus.PAUSED)
                    self._set_state(S.SAFETY_STOP)
                self.safety_stop_kind = kind
                self._commit()
            elif self.state == S.SAFETY_STOP:
                before = self._state_before_safety_stop
                # 移動中からの安全停止は自動で再開せず一時停止へ。それ以外は元の状態へ
                self._set_state(S.PAUSED if before == S.NAVIGATING else before)
                self.safety_stop_kind = None
                self._state_before_safety_stop = None
                self._commit()

    # ================================================================
    # 内部
    # ================================================================

    def _task_source_for(self, role):
        """移動指示の受付元はモードで決まる（上位指示モード: 上位システム, UI 指示モード: オペレータ）"""
        if self.mode == Mode.UPPER and role == Role.UPPER_SYSTEM:
            return TaskSource.UPPER
        if self.mode == Mode.OPERATOR and role in OPERATOR_ROLES:
            return TaskSource.OPERATOR
        raise PreconditionFailed("現在のモードではこの利用者からの移動指示を受け付けません", self.state, self.mode)

    def _require_role(self, role, allowed):
        if role not in allowed:
            raise PermissionDenied(f"役割 {role.name} はこの操作を実行できません")

    def _require_state(self, command):
        if self.state not in ACCEPTED_STATES[command]:
            raise PreconditionFailed(f"{self.state.name} では {command} を受け付けません", self.state, self.mode)

    def _require_task(self, task_id):
        if self.task is None or self.task.task_id != task_id:
            if task_id in self.tasks:
                raise PreconditionFailed(f"タスク {task_id} は保持中ではありません", self.state, self.mode)
            raise NotFound(f"タスク {task_id} はありません")
        return self.task

    def _idle_or_not_ready(self):
        return S.IDLE if self.active_map_id is not None and self.localized else S.NOT_READY

    def _navigate(self):
        self._goal_seq += 1
        self._goal_start_index = self.task.current_index
        self.nav_activity = NavActivity.NORMAL
        self._emit(
            NavigateThroughPoints(
                goal_seq=self._goal_seq, map_id=self.task.map_id, point_ids=tuple(self.task.remaining_point_ids)
            )
        )

    def _is_current_goal(self, goal_seq):
        return self.state == S.NAVIGATING and self.task is not None and goal_seq == self._goal_seq

    def _enter_fault(self, fault, cancel):
        if cancel:
            self._emit(CancelNavigation())
        self.fault = fault
        self._set_state(S.FAULT)
        self._set_task_status(TaskStatus.FAULTED, fault)
        self._commit()

    def _set_state(self, state):
        self.state = state
        if state != S.FAULT:
            self.fault = None

    def _set_task_status(self, status, fault=None):
        if self.task is None:
            return
        self.task.status = status
        self.task.history.append(TaskEvent(at=self._clock(), status=status, robot_state=self.state, fault=fault))

    def _finish_task(self, status):
        if self.task is None:
            return
        self._set_task_status(status)
        self.task.finished_at = self._clock()
        self.task = None
        self.fault = None

    def _emit(self, effect):
        self._on_effect(effect)

    def _commit(self):
        self.sequence += 1
        self.updated_at = self._clock()
        self._on_status(self.status())
