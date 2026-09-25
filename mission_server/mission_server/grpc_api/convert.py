"""中核のデータ型 <-> proto（locomotion.v1）の変換。"""

from google.protobuf.timestamp_pb2 import Timestamp
from locomotion.v1 import locomotion_pb2 as pb

from ..core import model
from ..store import Point


def timestamp(seconds: float | None) -> Timestamp | None:
    if seconds is None:
        return None
    ts = Timestamp()
    ts.FromNanoseconds(int(seconds * 1e9))
    return ts


def to_seconds(ts: Timestamp) -> float:
    return ts.ToNanoseconds() * 1e-9


# 列挙型: 中核の名前と proto の接頭辞付きの名前（例: RobotState.IDLE <-> ROBOT_STATE_IDLE）
_PREFIX = {
    model.RobotState: "ROBOT_STATE",
    model.Mode: "MODE",
    model.NavActivity: "NAV_ACTIVITY",
    model.SafetyStopKind: "SAFETY_STOP_KIND",
    model.FaultCategory: "FAULT_CATEGORY",
    model.TaskSource: "TASK_SOURCE",
    model.TaskStatus: "TASK_STATUS",
}


def enum_to_pb(value) -> int:
    if value is None:
        return 0  # *_UNSPECIFIED
    return getattr(pb, f"{_PREFIX[type(value)]}_{value.name}")


def enum_from_pb(enum_type, number: int):
    prefix = _PREFIX[enum_type] + "_"
    for name, value in pb.__dict__.items():
        if name.startswith(prefix) and value == number and isinstance(value, int):
            member = name[len(prefix) :]
            if member in enum_type.__members__:
                return enum_type[member]
    raise ValueError(f"{enum_type.__name__} に対応しない値です: {number}")


def pose_to_pb(pose: model.Pose2D) -> pb.Pose2D:
    return pb.Pose2D(x=pose.x, y=pose.y, yaw=pose.yaw)


def pose_from_pb(pose: pb.Pose2D) -> model.Pose2D:
    return model.Pose2D(pose.x, pose.y, pose.yaw)


def fault_to_pb(fault: model.Fault | None) -> pb.Fault | None:
    if fault is None:
        return None
    return pb.Fault(
        category=enum_to_pb(fault.category),
        detail_code=fault.detail_code,
        detail_message=fault.detail_message,
        occurred_at=timestamp(fault.occurred_at),
    )


def task_to_pb(task: model.Task, map_version: int = 0, with_history: bool = True) -> pb.Task:
    msg = pb.Task(
        task_id=task.task_id,
        request_id=task.request_id,
        source=enum_to_pb(task.source),
        map_id=task.map_id,
        map_version=map_version,
        point_ids=task.point_ids,
        current_index=task.current_index,
        status=enum_to_pb(task.status),
        created_at=timestamp(task.created_at),
        finished_at=timestamp(task.finished_at),
    )
    if with_history:
        for event in task.history:
            msg.history.append(
                pb.TaskEvent(
                    at=timestamp(event.at),
                    status=enum_to_pb(event.status),
                    robot_state=enum_to_pb(event.robot_state),
                    fault=fault_to_pb(event.fault),
                )
            )
    return msg


def status_to_pb(status: model.RobotStatus, localization, map_version: int) -> pb.RobotStatus:
    """localization: ros_adapter.LocalizationInfo（または同じ属性を持つもの）"""
    return pb.RobotStatus(
        sequence=status.sequence,
        updated_at=timestamp(status.updated_at),
        state=enum_to_pb(status.state),
        mode=enum_to_pb(status.mode),
        nav_activity=enum_to_pb(status.nav_activity),
        safety_stop_kind=enum_to_pb(status.safety_stop_kind),
        fault=fault_to_pb(status.fault),
        active_map_id=status.active_map_id or "",
        active_map_version=map_version,
        localization=pb.LocalizationStatus(
            localized=status.localized,
            covariance_xy=localization.std_xy,
            covariance_yaw=localization.std_yaw,
            applied_level=getattr(pb, f"THRESHOLD_LEVEL_{localization.level}"),
        ),
        joystick_accepting=status.joystick_accepting,
        active_task=task_to_pb(status.active_task, map_version, with_history=False) if status.active_task else None,
    )


def point_to_pb(point: Point) -> pb.Point:
    return pb.Point(
        map_id=point.map_id,
        point_id=point.point_id,
        name=point.name,
        pose=pose_to_pb(point.pose),
        move_mode_id=point.move_mode_id,
        tolerance=pb.Tolerance(xy=point.tolerance_xy, yaw=point.tolerance_yaw),
        kind=getattr(pb, f"POINT_KIND_{point.kind}", pb.POINT_KIND_UNSPECIFIED),
        needs_recheck=point.needs_recheck,
        version=point.version,
    )


def point_kind_from_pb(kind: int) -> str:
    name = pb.PointKind.Name(kind)
    return "WORK" if name == "POINT_KIND_UNSPECIFIED" else name.removeprefix("POINT_KIND_")
