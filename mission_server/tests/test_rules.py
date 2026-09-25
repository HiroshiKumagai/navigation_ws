"""受理・拒否の規則、権限、冪等性、観測の扱いのテスト（仕様書 3〜5 章、8 章の共通方針）。"""

import pytest

from conftest import INTEGRATOR, OP, UPPER, make_idle, make_navigating
from mission_server.core.model import (
    CancelNavigation,
    Fault,
    FaultCategory,
    Mode,
    NavActivity,
    NotFound,
    PermissionDenied,
    Pose2D,
    PreconditionFailed,
    ResetSafetyStop,
    RobotState as S,
    SafetyStopKind,
    TaskStatus,
    TeleopCommand,
)
from mission_server.core.state_machine import ACCEPTED_STATES

BLOCKED = Fault(FaultCategory.PATH_BLOCKED)


def bring_to(core, rec, state):
    """指定の状態にする。タスクがあればその task_id を返す"""
    if state == S.NOT_READY:
        return "none"
    if state in (S.MANUAL, S.MAPPING):
        make_idle(core, rec)
        core.set_mode(OP, Mode.JOYSTICK)
        if state == S.MAPPING:
            core.start_mapping(INTEGRATOR, "new_map", rescan=False)
        return "none"
    if state == S.IDLE:
        make_idle(core, rec)
        return "none"
    task = make_navigating(core, rec)
    if state == S.PAUSED:
        core.pause_task(OP, task.task_id)
    elif state == S.FAULT:
        core.on_navigation_failed(1, BLOCKED)
    elif state == S.SAFETY_STOP:
        core.on_safety_signal(1)
    return task.task_id


COMMANDS = {
    "start_task": lambda core, task_id: core.start_task(OP, "new", ["A"]),
    "pause_task": lambda core, task_id: core.pause_task(OP, task_id),
    "resume_task": lambda core, task_id: core.resume_task(OP, task_id),
    "stop_task": lambda core, task_id: core.stop_task(OP, task_id),
    "retry_task": lambda core, task_id: core.retry_task(OP, task_id),
    "set_mode": lambda core, task_id: core.set_mode(OP, Mode.UPPER if core.mode != Mode.UPPER else Mode.OPERATOR),
    "set_initial_pose": lambda core, task_id: core.set_initial_pose(OP, pose=Pose2D(0, 0, 0)),
    "reset_safety_stop": lambda core, task_id: core.reset_safety_stop(OP),
    "activate_map": lambda core, task_id: core.activate_map(OP, "m1"),
    "start_mapping": lambda core, task_id: core.start_mapping(INTEGRATOR, "another_map", rescan=False),
    "save_mapping": lambda core, task_id: core.save_mapping(INTEGRATOR),
    "discard_mapping": lambda core, task_id: core.discard_mapping(INTEGRATOR),
}
REJECTED_CASES = [(cmd, state) for cmd in COMMANDS for state in S if state not in ACCEPTED_STATES[cmd]]


@pytest.mark.parametrize("command,state", REJECTED_CASES, ids=[f"{c}-{s.name}" for c, s in REJECTED_CASES])
def test_command_rejected_outside_accepted_states(core, rec, command, state):
    """受け付けない状態では FAILED_PRECONDITION（現在の状態とモードを添える）で拒否し、状態を変えない"""
    task_id = bring_to(core, rec, state)
    assert core.state == state
    sequence = core.sequence
    rec.take_effects()
    with pytest.raises((PreconditionFailed, NotFound)) as err:
        COMMANDS[command](core, task_id)
    if isinstance(err.value, PreconditionFailed):
        assert err.value.state == state
        assert err.value.mode == core.mode
    assert core.state == state
    assert core.sequence == sequence
    assert rec.effects == []


@pytest.mark.parametrize("state", [S.NAVIGATING, S.PAUSED, S.SAFETY_STOP, S.MAPPING])
def test_mode_switch_forbidden_while_moving_paused_safety_or_mapping(core, rec, state):
    """モード切替は準備未完了・待機・異常停止・手動操作のときのみ（仕様書 3 章）"""
    bring_to(core, rec, state)
    with pytest.raises(PreconditionFailed):
        core.set_mode(OP, Mode.JOYSTICK)


# ---- 権限（仕様書 3 章） ----


def test_upper_system_cannot_operate(core, rec):
    """上位システムができるのは移動指示と自身のタスクの停止だけ"""
    task = make_navigating(core, rec)
    for operation in (
        lambda: core.pause_task(UPPER, task.task_id),
        lambda: core.set_mode(UPPER, Mode.JOYSTICK),
        lambda: core.reset_safety_stop(UPPER),
    ):
        with pytest.raises(PermissionDenied):
            operation()
    core.stop_task(OP, task.task_id)
    with pytest.raises(PermissionDenied):
        core.activate_map(UPPER, "m2")


def test_mapping_requires_integrator(core, rec):
    core.set_mode(OP, Mode.JOYSTICK)
    with pytest.raises(PermissionDenied):
        core.start_mapping(OP, "new_map", rescan=False)


def test_start_task_source_depends_on_mode(core, rec):
    """上位指示モードでは上位システム、UI 指示モードではオペレータの移動指示だけを受け付ける"""
    make_idle(core, rec)
    with pytest.raises(PreconditionFailed):
        core.start_task(UPPER, "r1", ["A"])
    core.set_mode(OP, Mode.UPPER)
    with pytest.raises(PreconditionFailed):
        core.start_task(OP, "r2", ["A"])
    task = core.start_task(UPPER, "r3", ["A"])
    assert core.state == S.NAVIGATING
    # 上位システムは自身が出したタスクを停止できる
    core.stop_task(UPPER, task.task_id)
    assert task.status == TaskStatus.CANCELED


def test_upper_system_cannot_stop_operator_task(core, rec):
    task = make_navigating(core, rec)
    with pytest.raises(PermissionDenied):
        core.stop_task(UPPER, task.task_id)


def test_operator_can_pause_upper_task(core, rec):
    """人が止めたものは人が再開する。上位指示モード中でもオペレータが一時停止・再開できる"""
    make_idle(core, rec)
    core.set_mode(OP, Mode.UPPER)
    task = core.start_task(UPPER, "r1", ["A"])
    core.pause_task(OP, task.task_id)
    core.resume_task(OP, task.task_id)
    assert core.state == S.NAVIGATING


# ---- 移動指示（仕様書 8 章の共通方針） ----


def test_start_task_is_idempotent_by_request_id(core, rec):
    make_idle(core, rec)
    first = core.start_task(OP, "same", ["A"])
    effects = rec.take_effects()
    again = core.start_task(OP, "same", ["A"])  # 移動中でも同じ要求 ID なら既存タスクを返す
    assert again is first
    assert len(effects) == 1 and rec.effects == []


def test_start_task_rejects_other_map(core, rec):
    make_idle(core, rec)
    with pytest.raises(PreconditionFailed):
        core.start_task(OP, "r1", ["A"], expected_map_id="m2")


def test_start_task_rejects_unknown_point(core, rec):
    make_idle(core, rec)
    with pytest.raises(NotFound):
        core.start_task(OP, "r1", ["A", "Z"])
    assert core.state == S.IDLE


def test_command_to_unknown_task(core, rec):
    make_navigating(core, rec)
    with pytest.raises(NotFound):
        core.pause_task(OP, "no_such_task")


def test_stale_goal_results_are_ignored(core, rec):
    """一時停止でキャンセルした古い目標の結果は無視する"""
    task = make_navigating(core, rec)
    core.pause_task(OP, task.task_id)
    core.resume_task(OP, task.task_id)  # goal_seq=2
    core.on_navigation_failed(1, BLOCKED)
    core.on_navigation_succeeded(1)
    assert core.state == S.NAVIGATING
    core.on_navigation_succeeded(2)
    assert core.state == S.IDLE


def test_nav_activity_recovering(core, rec):
    make_navigating(core, rec)
    core.on_nav_activity(1, NavActivity.RECOVERING)
    assert core.status().nav_activity == NavActivity.RECOVERING
    core.on_nav_activity(1, NavActivity.NORMAL)
    assert core.status().nav_activity == NavActivity.NORMAL


# ---- 安全系 ----


def test_emergency_reset_is_requested_but_state_follows_signal(core, rec):
    make_navigating(core, rec)
    core.on_safety_signal(1)
    with pytest.raises(PreconditionFailed):
        core.reset_safety_stop(OP)  # 保護停止はリセット不要
    core.on_safety_signal(2)
    assert core.status().safety_stop_kind == SafetyStopKind.EMERGENCY
    rec.take_effects()
    core.reset_safety_stop(OP)
    assert rec.effects == [ResetSafetyStop()]
    assert core.state == S.SAFETY_STOP  # 安全系の信号が解除されるまでは安全停止のまま
    core.on_safety_signal(0)
    assert core.state == S.PAUSED


def test_repeated_safety_signal_does_not_notify(core, rec):
    make_idle(core, rec)
    core.on_safety_signal(1)
    sequence = core.sequence
    core.on_safety_signal(1)
    assert core.sequence == sequence


# ---- 異常の検知（仕様書 5 章） ----


def test_localization_lost_while_navigating_is_fault(core, rec):
    task = make_navigating(core, rec)
    core.on_localization(False)
    assert core.state == S.FAULT
    assert core.status().fault.category == FaultCategory.LOCALIZATION_LOST
    assert rec.effects == [CancelNavigation()]
    # 初期位置の再設定で待機へ（タスクは失敗）
    core.set_initial_pose(OP, point_id="A")
    assert core.state == S.IDLE and task.status == TaskStatus.FAILED


def test_device_fault_while_navigating(core, rec):
    make_navigating(core, rec)
    core.on_device_fault(Fault(FaultCategory.DEVICE_ERROR, "lidar_front_left"))
    assert core.state == S.FAULT
    assert rec.effects == [CancelNavigation()]


def test_status_sequence_increases_on_every_change(core, rec):
    make_navigating(core, rec)
    sequences = [s.sequence for s in rec.statuses]
    assert sequences == sorted(sequences) and len(set(sequences)) == len(sequences)


# ---- 実装で見つかり、仕様書に反映した挙動（Sep 25 改訂）と未決事項 ----


def test_localization_lost_while_idle_goes_not_ready(core, rec):
    """待機中の自己位置喪失は準備未完了へ（初期位置の再設定ができるように）"""
    make_idle(core, rec)
    core.on_localization(False)
    assert core.state == S.NOT_READY


def test_stop_from_fault_with_lost_localization_goes_not_ready(core, rec):
    """自己位置喪失の異常停止から停止指示で抜けた場合は準備未完了へ"""
    task = make_navigating(core, rec)
    core.on_localization(False)
    core.stop_task(OP, task.task_id)
    assert core.state == S.NOT_READY


def test_switch_between_navigation_modes_in_fault(core, rec):
    """異常停止中に上位指示 <-> UI 指示へ切り替えると、タスクを破棄（失敗）して待機へ"""
    task = make_navigating(core, rec)
    core.on_navigation_failed(1, BLOCKED)
    assert core.set_mode(OP, Mode.UPPER) == S.IDLE
    assert task.status == TaskStatus.FAILED


def test_undecided_stop_rejected_during_safety_stop(core, rec):
    """安全停止中の停止指示（タスクの破棄）は、仕様書 10 章の未決事項が決まるまで受け付けない"""
    task = make_navigating(core, rec)
    core.on_safety_signal(1)
    with pytest.raises(PreconditionFailed):
        core.stop_task(OP, task.task_id)


def test_task_history_records_state_after_transition(core, rec):
    """実行記録の各行には、その出来事のあとのロボットの状態を残す"""
    task = make_navigating(core, rec)
    core.on_safety_signal(1)
    core.on_safety_signal(0)
    core.resume_task(OP, task.task_id)
    core.on_navigation_succeeded(2)
    assert [(e.status, e.robot_state) for e in task.history] == [
        (TaskStatus.RUNNING, S.NAVIGATING),
        (TaskStatus.PAUSED, S.SAFETY_STOP),
        (TaskStatus.RUNNING, S.NAVIGATING),
        (TaskStatus.SUCCEEDED, S.IDLE),
    ]


def test_teleop_only_in_joystick_manual(core, rec):
    """操作パッドはジョイスティックモードで手動操作・地図作成中のみ（状態は変えない）"""
    make_idle(core, rec)
    with pytest.raises(PreconditionFailed):
        core.teleop(OP, 0.3, 0.0, 0.0)
    core.set_mode(OP, Mode.JOYSTICK)
    rec.take_effects()
    sequence = core.sequence
    core.teleop(OP, 0.3, -0.1, 0.2)
    assert rec.effects == [TeleopCommand(0.3, -0.1, 0.2)]
    assert core.sequence == sequence
    with pytest.raises(PermissionDenied):
        core.teleop(UPPER, 0.3, 0.0, 0.0)
