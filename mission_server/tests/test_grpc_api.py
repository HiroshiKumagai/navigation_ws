"""gRPC API のテスト（ROS 2 アダプタは偽物に差し替え、実際の gRPC サーバとクライアントで通信する）。"""

import shutil
import threading
from pathlib import Path

import grpc
import pytest
from grpc_status import rpc_status
from locomotion.v1 import locomotion_pb2 as pb
from locomotion.v1 import locomotion_pb2_grpc as pb_grpc

from mission_server.core.model import NavigateThroughPoints, Pose2D
from mission_server.ros_adapter import LocalizationInfo, RobotPose
from mission_server.server import build, serve
from mission_server.store import Store

DATA_DIR = Path(__file__).resolve().parents[2] / "data"
OPERATOR = [("x-locomotion-role", "operator")]
UPPER = [("x-locomotion-role", "upper-system")]
INTEGRATOR = [("x-locomotion-role", "integrator")]


class FakeAdapter:
    def __init__(self):
        self.effects = []
        self.core = None

    def attach(self, core):
        self.core = core

    def handle_effect(self, effect):
        self.effects.append(effect)

    def latest_pose(self):
        return RobotPose(stamp=12.5, pose=Pose2D(1.0, 2.0, 0.5), linear_x=0.1, linear_y=0.2, angular=0.0)

    def localization(self):
        return LocalizationInfo(True, 0.05, 0.02, "STANDARD")

    def last_goal_seq(self):
        return [e for e in self.effects if isinstance(e, NavigateThroughPoints)][-1].goal_seq


@pytest.fixture
def api(tmp_path):
    data = tmp_path / "data"
    shutil.copytree(DATA_DIR, data)
    adapter = FakeAdapter()
    ctx = build(Store(data), adapter)
    server, port = serve(ctx, 0, max_workers=8)
    channel = grpc.insecure_channel(f"localhost:{port}")

    class Api:
        robot = pb_grpc.RobotServiceStub(channel)
        task = pb_grpc.TaskServiceStub(channel)
        maps = pb_grpc.MapServiceStub(channel)
        points = pb_grpc.PointServiceStub(channel)

    Api.ctx, Api.adapter = ctx, adapter
    yield Api
    channel.close()
    server.stop(grace=None)


def make_idle(api):
    api.robot.SetInitialPose(pb.SetInitialPoseRequest(point_id="home"), metadata=OPERATOR)


def start(api, points=("A",), request_id="r1", metadata=OPERATOR):
    return api.task.StartTask(pb.StartTaskRequest(request_id=request_id, point_ids=points), metadata=metadata).task


def test_role_metadata_is_required(api):
    with pytest.raises(grpc.RpcError) as err:
        api.robot.GetStatus(pb.GetStatusRequest())
    assert err.value.code() == grpc.StatusCode.UNAUTHENTICATED


def test_initial_status(api):
    status = api.robot.GetStatus(pb.GetStatusRequest(), metadata=OPERATOR)
    assert status.state == pb.ROBOT_STATE_NOT_READY
    assert status.mode == pb.MODE_OPERATOR
    assert status.active_map_id == "warehouse" and status.active_map_version == 1
    assert status.localization.applied_level == pb.THRESHOLD_LEVEL_STANDARD


def test_task_lifecycle(api):
    make_idle(api)
    task = start(api, points=("A", "B"))
    assert task.status == pb.TASK_STATUS_RUNNING and list(task.point_ids) == ["A", "B"]
    assert api.robot.GetStatus(pb.GetStatusRequest(), metadata=OPERATOR).state == pb.ROBOT_STATE_NAVIGATING

    paused = api.task.PauseTask(pb.TaskCommandRequest(task_id=task.task_id), metadata=OPERATOR).task
    assert paused.status == pb.TASK_STATUS_PAUSED
    api.task.ResumeTask(pb.TaskCommandRequest(task_id=task.task_id), metadata=OPERATOR)

    # Nav2 の成功（アダプタからの観測）
    api.ctx.core.on_navigation_succeeded(api.adapter.last_goal_seq())
    done = api.task.GetTask(pb.GetTaskRequest(task_id=task.task_id), metadata=OPERATOR)
    assert done.status == pb.TASK_STATUS_SUCCEEDED
    assert [e.status for e in done.history] == [
        pb.TASK_STATUS_RUNNING,
        pb.TASK_STATUS_PAUSED,
        pb.TASK_STATUS_RUNNING,
        pb.TASK_STATUS_SUCCEEDED,
    ]
    listed = api.task.ListTasks(pb.ListTasksRequest(), metadata=OPERATOR)
    assert [t.task_id for t in listed.tasks] == [task.task_id]


def test_start_task_is_idempotent(api):
    make_idle(api)
    first = start(api, request_id="same")
    again = start(api, request_id="same")
    assert again.task_id == first.task_id


def test_failed_precondition_has_rejection_detail(api):
    make_idle(api)
    start(api)
    with pytest.raises(grpc.RpcError) as err:
        api.robot.SetMode(pb.SetModeRequest(mode=pb.MODE_JOYSTICK), metadata=OPERATOR)
    assert err.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    status = rpc_status.from_call(err.value)
    detail = pb.RejectionDetail()
    status.details[0].Unpack(detail)
    assert detail.current_state == pb.ROBOT_STATE_NAVIGATING
    assert detail.current_mode == pb.MODE_OPERATOR
    assert detail.reason


def test_permission_denied_and_not_found(api):
    make_idle(api)
    with pytest.raises(grpc.RpcError) as err:
        api.robot.SetMode(pb.SetModeRequest(mode=pb.MODE_JOYSTICK), metadata=UPPER)
    assert err.value.code() == grpc.StatusCode.PERMISSION_DENIED
    with pytest.raises(grpc.RpcError) as err:
        start(api, points=("A", "NO_SUCH_POINT"))
    assert err.value.code() == grpc.StatusCode.NOT_FOUND


def test_set_mode_returns_resulting_state(api):
    make_idle(api)
    response = api.robot.SetMode(pb.SetModeRequest(mode=pb.MODE_JOYSTICK), metadata=OPERATOR)
    assert response.resulting_state == pb.ROBOT_STATE_MANUAL
    response = api.robot.SetMode(pb.SetModeRequest(mode=pb.MODE_OPERATOR), metadata=OPERATOR)
    assert response.resulting_state == pb.ROBOT_STATE_IDLE


def test_watch_status_sends_current_then_changes(api):
    stream = api.robot.WatchStatus(pb.WatchStatusRequest(), metadata=OPERATOR)
    first = next(stream)
    assert first.state == pb.ROBOT_STATE_NOT_READY

    threading.Timer(0.2, make_idle, args=(api,)).start()
    second = next(stream)
    assert second.state == pb.ROBOT_STATE_IDLE
    assert second.sequence > first.sequence
    stream.cancel()


def test_watch_pose(api):
    stream = api.robot.WatchPose(pb.WatchPoseRequest(rate_hz=20), metadata=OPERATOR)
    update = next(stream)
    assert (update.pose.x, update.pose.y, update.linear_velocity_y) == pytest.approx((1.0, 2.0, 0.2))
    assert update.map_id == "warehouse"
    stream.cancel()


def test_points(api):
    listed = api.points.ListPoints(pb.ListPointsRequest(map_id="warehouse"), metadata=OPERATOR)
    assert {p.point_id for p in listed.points} >= {"home", "A", "B"}

    request = pb.CreatePointRequest(map_id="warehouse", point_id="P1", pose=pb.Pose2D(x=1, y=1, yaw=0))
    with pytest.raises(grpc.RpcError) as err:
        api.points.CreatePoint(request, metadata=OPERATOR)
    assert err.value.code() == grpc.StatusCode.PERMISSION_DENIED
    created = api.points.CreatePoint(request, metadata=INTEGRATOR)
    assert created.point_id == "P1" and created.kind == pb.POINT_KIND_WORK

    # ティーチング（現在位置の記録）には自己位置の確定が必要
    teach = pb.CreatePointRequest(map_id="warehouse", point_id="P2", use_current_pose=True)
    with pytest.raises(grpc.RpcError) as err:
        api.points.CreatePoint(teach, metadata=INTEGRATOR)
    assert err.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    make_idle(api)
    taught = api.points.CreatePoint(teach, metadata=INTEGRATOR)
    assert (taught.pose.x, taught.pose.y) == pytest.approx((1.0, 2.0))
    # 登録した地点へ移動指示できる
    assert start(api, points=("P2",)).status == pb.TASK_STATUS_RUNNING


def test_maps(api):
    maps = api.maps.ListMaps(pb.ListMapsRequest(), metadata=OPERATOR).maps
    assert [m.map_id for m in maps] == ["warehouse"]
    api.maps.ActivateMap(pb.ActivateMapRequest(map_id="warehouse"), metadata=OPERATOR)
    assert api.robot.GetStatus(pb.GetStatusRequest(), metadata=OPERATOR).state == pb.ROBOT_STATE_NOT_READY


def test_teleop(api):
    make_idle(api)
    with pytest.raises(grpc.RpcError) as err:
        api.robot.Teleop(pb.TeleopRequest(linear_velocity_x=0.3), metadata=OPERATOR)
    assert err.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    api.robot.SetMode(pb.SetModeRequest(mode=pb.MODE_JOYSTICK), metadata=OPERATOR)
    api.robot.Teleop(pb.TeleopRequest(linear_velocity_x=0.3, angular_velocity=-0.2), metadata=OPERATOR)
    assert api.adapter.effects[-1].linear_x == pytest.approx(0.3)
