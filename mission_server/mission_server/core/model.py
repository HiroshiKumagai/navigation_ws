"""状態管理の中核で使うデータ型（仕様書 3〜5 章）。ROS 2 にも gRPC にも依存しない。"""

from dataclasses import dataclass, field
from enum import Enum, auto


class RobotState(Enum):
    """ロボットのナビゲーション状態（仕様書 4 章）"""

    NOT_READY = auto()  # 準備未完了
    IDLE = auto()  # 待機
    NAVIGATING = auto()  # 移動中
    PAUSED = auto()  # 一時停止
    FAULT = auto()  # 異常停止
    SAFETY_STOP = auto()  # 安全停止
    MANUAL = auto()  # 手動操作
    MAPPING = auto()  # 地図作成中


class Mode(Enum):
    """誰が指示するか（仕様書 3 章）"""

    UPPER = auto()  # 上位指示モード
    OPERATOR = auto()  # UI 指示モード
    JOYSTICK = auto()  # ジョイスティックモード


class Role(Enum):
    """API 利用者の役割（仕様書 3 章）"""

    UPPER_SYSTEM = auto()  # 上位システム
    OPERATOR = auto()  # オペレータ
    INTEGRATOR = auto()  # 導入担当者（オペレータの操作をすべて含む）


class NavActivity(Enum):
    NORMAL = auto()
    RECOVERING = auto()  # 回避動作中（Nav2 のリカバリ実行中）


class SafetyStopKind(Enum):
    PROTECTIVE = auto()  # 保護停止
    EMERGENCY = auto()  # 非常停止


class FaultCategory(Enum):
    """異常の分類（仕様書 5 章）"""

    PATH_BLOCKED = auto()  # 経路閉塞
    GOAL_UNREACHABLE = auto()  # 目標到達不能
    LOCALIZATION_LOST = auto()  # 自己位置喪失
    DEVICE_ERROR = auto()  # 機器異常


class TaskSource(Enum):
    UPPER = auto()
    OPERATOR = auto()


class TaskStatus(Enum):
    RUNNING = auto()
    PAUSED = auto()  # 一時停止中、または移動中から安全停止に入った（保持中）
    FAULTED = auto()  # 異常停止中（保持中）
    SUCCEEDED = auto()  # 成功
    CANCELED = auto()  # 中断
    FAILED = auto()  # 失敗

    @property
    def finished(self):
        return self in (TaskStatus.SUCCEEDED, TaskStatus.CANCELED, TaskStatus.FAILED)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Fault:
    category: FaultCategory
    detail_code: str = ""
    detail_message: str = ""
    occurred_at: float = 0.0


@dataclass(frozen=True)
class TaskEvent:
    at: float
    status: TaskStatus
    robot_state: RobotState
    fault: Fault | None = None


@dataclass
class Task:
    task_id: str
    request_id: str
    source: TaskSource
    map_id: str
    point_ids: list[str]
    created_at: float
    current_index: int = 0  # 巡回中の現在の目標（point_ids の添字）
    status: TaskStatus = TaskStatus.RUNNING
    history: list[TaskEvent] = field(default_factory=list)
    finished_at: float | None = None
    route_id: str = ""  # ルートで指示した場合（point_ids は開始時点のルートの内容）

    @property
    def remaining_point_ids(self):
        return self.point_ids[self.current_index :]


@dataclass(frozen=True)
class RobotStatus:
    """配信する状態のスナップショット（proto の RobotStatus に対応）"""

    sequence: int
    updated_at: float
    state: RobotState
    mode: Mode
    nav_activity: NavActivity | None
    safety_stop_kind: SafetyStopKind | None
    fault: Fault | None
    active_map_id: str | None
    localized: bool
    joystick_accepting: bool
    active_task: Task | None


# ---- 副作用: 中核が決め、ROS 2 アダプタが実行する ----


@dataclass(frozen=True)
class Effect:
    pass


@dataclass(frozen=True)
class NavigateThroughPoints(Effect):
    """Nav2 に目標を送る。goal_seq は結果の突き合わせ用（古い目標の結果は捨てる）"""

    goal_seq: int
    map_id: str
    point_ids: tuple[str, ...]


@dataclass(frozen=True)
class CancelNavigation(Effect):
    pass


@dataclass(frozen=True)
class SetCommandMode(Effect):
    """指令切替ノードのモード（True でジョイスティックの指令を通す）"""

    joystick: bool


@dataclass(frozen=True)
class TeleopCommand(Effect):
    """操作パッドの速度指令（ロボット座標系）"""

    linear_x: float
    linear_y: float
    angular: float


@dataclass(frozen=True)
class SetInitialPose(Effect):
    pose: Pose2D | None = None
    point_id: str | None = None


@dataclass(frozen=True)
class ResetSafetyStop(Effect):
    pass


@dataclass(frozen=True)
class ActivateMap(Effect):
    map_id: str


@dataclass(frozen=True)
class StartMapping(Effect):
    map_id: str
    rescan: bool


@dataclass(frozen=True)
class FinishMapping(Effect):
    save: bool


# ---- 拒否 ----


class Rejected(Exception):
    """API の指示を受け付けない。gRPC 層でステータスコードに変換する"""


class PreconditionFailed(Rejected):
    """状態・モードが許さない（FAILED_PRECONDITION）"""

    def __init__(self, reason, state, mode):
        super().__init__(reason)
        self.reason = reason
        self.state = state
        self.mode = mode


class PermissionDenied(Rejected):
    """役割が足りない（PERMISSION_DENIED）"""


class NotFound(Rejected):
    """対象がない（NOT_FOUND）"""
