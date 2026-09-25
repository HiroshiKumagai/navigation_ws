"""仕様書 4 章「遷移表」の各行のテスト。テスト名の番号は遷移表の行の順番。"""

import pytest

from conftest import OP, INTEGRATOR, make_idle, make_navigating
from mission_server.core.model import (
    ActivateMap,
    CancelNavigation,
    Fault,
    FaultCategory,
    FinishMapping,
    Mode,
    NavigateThroughPoints,
    Pose2D,
    RobotState as S,
    SafetyStopKind,
    SetCommandMode,
    SetInitialPose,
    StartMapping,
    TaskStatus,
)

BLOCKED = Fault(FaultCategory.PATH_BLOCKED, "NAV2_FAILED")


def goal_seq_of(effects):
    return [e for e in effects if isinstance(e, NavigateThroughPoints)][-1].goal_seq


def test_01_not_ready_to_idle_by_map_and_initial_pose(core, rec):
    """準備未完了 -> 待機: 地図の有効化と初期位置の設定"""
    assert core.state == S.NOT_READY
    core.activate_map(OP, "m1")
    assert core.state == S.NOT_READY
    core.set_initial_pose(OP, pose=Pose2D(1.0, 2.0, 0.0))
    assert core.state == S.IDLE
    assert rec.effects == [ActivateMap("m1"), SetInitialPose(pose=Pose2D(1.0, 2.0, 0.0))]


def test_02_idle_to_navigating_by_start_task(core, rec):
    """待機 -> 移動中: 移動指示（現在のモードの指示元から）"""
    make_idle(core, rec)
    task = core.start_task(OP, "r1", ["A", "B"])
    assert core.state == S.NAVIGATING
    assert task.status == TaskStatus.RUNNING
    assert rec.effects == [NavigateThroughPoints(goal_seq=1, map_id="m1", point_ids=("A", "B"))]


def test_03a_navigating_to_idle_by_goal_reached(core, rec):
    """移動中 -> 待機: 目標に到達（結果は成功）"""
    task = make_navigating(core, rec, points=("A", "B"))
    core.on_waypoint_reached(1, 0)
    assert task.current_index == 1
    core.on_navigation_succeeded(1)
    assert core.state == S.IDLE
    assert task.status == TaskStatus.SUCCEEDED
    assert core.task is None


def test_03b_navigating_to_idle_by_stop(core, rec):
    """移動中 -> 待機: 停止指示（結果は中断）"""
    task = make_navigating(core, rec)
    core.stop_task(OP, task.task_id)
    assert core.state == S.IDLE
    assert task.status == TaskStatus.CANCELED
    assert rec.effects == [CancelNavigation()]


def test_04_navigating_to_paused(core, rec):
    """移動中 -> 一時停止: 一時停止指示"""
    task = make_navigating(core, rec)
    core.pause_task(OP, task.task_id)
    assert core.state == S.PAUSED
    assert task.status == TaskStatus.PAUSED
    assert rec.effects == [CancelNavigation()]


def test_05_paused_to_navigating_resends_remaining_points(core, rec):
    """一時停止 -> 移動中: 再開指示（残りの地点を送り直す）"""
    task = make_navigating(core, rec, points=("A", "B", "C"))
    core.on_waypoint_reached(1, 0)
    core.pause_task(OP, task.task_id)
    rec.take_effects()
    core.resume_task(OP, task.task_id)
    assert core.state == S.NAVIGATING
    assert rec.effects == [NavigateThroughPoints(goal_seq=2, map_id="m1", point_ids=("B", "C"))]


def test_06_paused_to_idle_by_stop(core, rec):
    """一時停止 -> 待機: 停止指示（タスクを破棄、結果は中断）"""
    task = make_navigating(core, rec)
    core.pause_task(OP, task.task_id)
    rec.take_effects()
    core.stop_task(OP, task.task_id)
    assert core.state == S.IDLE
    assert task.status == TaskStatus.CANCELED
    assert rec.effects == []  # 一時停止中は Nav2 の目標がないのでキャンセル不要


def test_07_navigating_to_fault(core, rec):
    """移動中 -> 異常停止: 異常の検知"""
    task = make_navigating(core, rec)
    core.on_navigation_failed(1, BLOCKED)
    assert core.state == S.FAULT
    assert core.status().fault == BLOCKED
    assert task.status == TaskStatus.FAULTED
    assert core.task is task  # タスクは保持したまま


def test_08_fault_to_navigating_by_retry(core, rec):
    """異常停止 -> 移動中: 再試行（保持していたタスクを再送）"""
    task = make_navigating(core, rec, points=("A", "B"))
    core.on_navigation_failed(1, BLOCKED)
    core.retry_task(OP, task.task_id)
    assert core.state == S.NAVIGATING
    assert core.status().fault is None
    assert rec.effects == [NavigateThroughPoints(goal_seq=2, map_id="m1", point_ids=("A", "B"))]


def test_09a_fault_to_idle_by_initial_pose(core, rec):
    """異常停止 -> 待機: 初期位置の再設定（タスクを破棄、結果は失敗）"""
    task = make_navigating(core, rec)
    core.on_navigation_failed(1, Fault(FaultCategory.LOCALIZATION_LOST))
    core.set_initial_pose(OP, point_id="B")
    assert core.state == S.IDLE
    assert task.status == TaskStatus.FAILED


def test_09b_fault_to_idle_by_stop(core, rec):
    """異常停止 -> 待機: 停止指示（タスクを破棄、結果は失敗）"""
    task = make_navigating(core, rec)
    core.on_navigation_failed(1, BLOCKED)
    core.stop_task(OP, task.task_id)
    assert core.state == S.IDLE
    assert task.status == TaskStatus.FAILED


def test_10_navigating_to_safety_stop_cancels_goal(core, rec):
    """移動中 -> 安全停止: 安全系からの停止信号（直ちに Nav2 の目標をキャンセル、タスクは一時停止扱い）"""
    task = make_navigating(core, rec)
    core.on_safety_signal(1)
    assert core.state == S.SAFETY_STOP
    assert core.status().safety_stop_kind == SafetyStopKind.PROTECTIVE
    assert task.status == TaskStatus.PAUSED
    assert rec.effects == [CancelNavigation()]


def test_11_safety_stop_from_navigating_releases_to_paused(core, rec):
    """安全停止 -> 一時停止: 移動中から入った場合の、停止信号の解除（自動で再開しない）"""
    make_navigating(core, rec)
    core.on_safety_signal(2)
    rec.take_effects()
    core.on_safety_signal(0)
    assert core.state == S.PAUSED
    assert rec.effects == []


@pytest.mark.parametrize("setup", ["not_ready", "idle", "paused", "fault", "manual", "mapping"])
def test_12_13_safety_stop_from_other_states_returns_to_origin(core, rec, setup):
    """移動中以外のすべて -> 安全停止 -> 元の状態: 停止信号と、その解除"""
    if setup != "not_ready":
        task = make_navigating(core, rec)
        if setup == "idle":
            core.stop_task(OP, task.task_id)
        elif setup == "paused":
            core.pause_task(OP, task.task_id)
        elif setup == "fault":
            core.on_navigation_failed(1, BLOCKED)
        elif setup in ("manual", "mapping"):
            core.stop_task(OP, task.task_id)
            core.set_mode(OP, Mode.JOYSTICK)
            if setup == "mapping":
                core.start_mapping(INTEGRATOR, "new_map", rescan=False)
    before = core.state
    task_status = core.task.status if core.task else None
    rec.take_effects()

    core.on_safety_signal(1)
    assert core.state == S.SAFETY_STOP
    assert (core.task.status if core.task else None) == task_status  # タスクの状態は保つ
    core.on_safety_signal(0)
    assert core.state == before
    assert rec.effects == []  # 地図作成中も SLAM は止めない


@pytest.mark.parametrize("setup", ["not_ready", "idle"])
def test_14_not_ready_or_idle_to_manual(core, rec, setup):
    """準備未完了／待機 -> 手動操作: ジョイスティックモードへの切替"""
    if setup == "idle":
        make_idle(core, rec)
    core.set_mode(OP, Mode.JOYSTICK)
    assert core.state == S.MANUAL
    assert core.status().joystick_accepting
    assert rec.effects == [SetCommandMode(joystick=True)]


def test_15_fault_to_manual_discards_task(core, rec):
    """異常停止 -> 手動操作: ジョイスティックモードへの切替（タスクを破棄、結果は失敗）"""
    task = make_navigating(core, rec)
    core.on_navigation_failed(1, Fault(FaultCategory.LOCALIZATION_LOST))
    core.set_mode(OP, Mode.JOYSTICK)
    assert core.state == S.MANUAL
    assert task.status == TaskStatus.FAILED
    assert core.task is None


def test_16_manual_to_idle_when_localized(core, rec):
    """手動操作 -> 待機: ナビゲーションモードへの切替（自己位置が正常な場合）"""
    make_idle(core, rec)
    core.set_mode(OP, Mode.JOYSTICK)
    rec.take_effects()
    assert core.set_mode(OP, Mode.OPERATOR) == S.IDLE
    assert rec.effects == [SetCommandMode(joystick=False)]


@pytest.mark.parametrize("reason", ["no_map", "lost"])
def test_17_manual_to_not_ready(core, rec, reason):
    """手動操作 -> 準備未完了: ナビゲーションモードへの切替（地図が未選択、または自己位置を喪失している場合）"""
    if reason == "lost":
        make_idle(core, rec)
    core.set_mode(OP, Mode.JOYSTICK)
    if reason == "lost":
        core.on_localization(False)
    assert core.set_mode(OP, Mode.OPERATOR) == S.NOT_READY


@pytest.mark.parametrize("rescan", [False, True])
def test_18_manual_to_mapping(core, rec, rescan):
    """手動操作 -> 地図作成中: 地図作成の開始（新規、または既存地図 ID の再スキャン）"""
    core.set_mode(OP, Mode.JOYSTICK)
    rec.take_effects()
    map_id = "m1" if rescan else "new_map"
    core.start_mapping(INTEGRATOR, map_id, rescan=rescan)
    assert core.state == S.MAPPING
    assert core.status().joystick_accepting
    assert rec.effects == [StartMapping(map_id=map_id, rescan=rescan)]


def test_19a_mapping_to_manual_by_save_inherits_pose(core, rec):
    """地図作成中 -> 手動操作: 保存（SLAM の最終推定位置を初期位置として引き継ぐ -> 戻りで待機へ）"""
    core.set_mode(OP, Mode.JOYSTICK)
    core.start_mapping(INTEGRATOR, "new_map", rescan=False)
    rec.take_effects()
    assert core.save_mapping(INTEGRATOR) == "new_map"
    assert core.state == S.MANUAL
    assert rec.effects == [FinishMapping(save=True)]
    assert core.set_mode(OP, Mode.OPERATOR) == S.IDLE
    assert core.active_map_id == "new_map"


def test_19b_mapping_to_manual_by_discard_loses_pose(core, rec):
    """地図作成中 -> 手動操作: 破棄（自己位置を喪失した扱い -> 戻りで準備未完了へ）"""
    make_idle(core, rec)
    core.set_mode(OP, Mode.JOYSTICK)
    core.start_mapping(INTEGRATOR, "new_map", rescan=False)
    core.discard_mapping(INTEGRATOR)
    assert core.state == S.MANUAL
    assert core.set_mode(OP, Mode.OPERATOR) == S.NOT_READY


@pytest.mark.parametrize("setup", ["not_ready", "idle"])
def test_20_switch_map_to_not_ready(core, rec, setup):
    """準備未完了／待機 -> 準備未完了: 使用する地図の切替"""
    if setup == "idle":
        make_idle(core, rec)
    core.activate_map(OP, "m2")
    assert core.state == S.NOT_READY
    assert core.active_map_id == "m2"
    assert not core.localized
