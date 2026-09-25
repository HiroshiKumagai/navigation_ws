import itertools

import pytest

from mission_server.core.model import Mode, Pose2D, Role
from mission_server.core.state_machine import LocomotionCore

OP = Role.OPERATOR
UPPER = Role.UPPER_SYSTEM
INTEGRATOR = Role.INTEGRATOR
POINTS = {"m1": {"A", "B", "C"}, "m2": {"A"}}


class Recorder:
    """中核が出した副作用と状態通知を記録する"""

    def __init__(self):
        self.effects = []
        self.statuses = []

    def take_effects(self):
        # 中核は self.effects.append を持っているので、リストは差し替えずに中身を空にする
        effects = list(self.effects)
        self.effects.clear()
        return effects


@pytest.fixture
def rec():
    return Recorder()


@pytest.fixture
def core(rec):
    counter = itertools.count(1)
    clock = itertools.count(100)
    return LocomotionCore(
        initial_mode=Mode.OPERATOR,
        point_exists=lambda map_id, point_id: point_id in POINTS.get(map_id, set()),
        map_exists=lambda map_id: map_id in POINTS,
        on_effect=rec.effects.append,
        on_status=rec.statuses.append,
        clock=lambda: float(next(clock)),
        new_task_id=lambda: f"task{next(counter)}",
    )


def make_idle(core, rec):
    """準備未完了 -> 待機（地図の有効化と初期位置の設定）"""
    core.activate_map(OP, "m1")
    core.set_initial_pose(OP, pose=Pose2D(0.0, 0.0, 0.0))
    rec.take_effects()
    return core


def make_navigating(core, rec, points=("A",)):
    make_idle(core, rec)
    task = core.start_task(OP, "req1", list(points))
    rec.take_effects()
    return task
