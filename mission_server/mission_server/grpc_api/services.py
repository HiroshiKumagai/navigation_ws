"""gRPC サービス（仕様書 8 章）。中核への指示と、状態・位置の配信を受け持つ。

- 役割は metadata の x-locomotion-role で受け取る（upper-system / operator / integrator）。
  本番では Envoy で認証し、その結果を同じ metadata で渡す想定（仕様書 3 章）
- 中核の拒否は gRPC 標準ステータスに変換する。FAILED_PRECONDITION には RejectionDetail を添える
"""

import math
import queue
import threading
import time

import grpc
from google.protobuf import any_pb2
from google.rpc import code_pb2, status_pb2
from grpc_status import rpc_status
from locomotion.v1 import locomotion_pb2 as pb
from locomotion.v1 import locomotion_pb2_grpc as pb_grpc

from ..core import model
from ..core.model import NotFound, PermissionDenied, PreconditionFailed, Rejected, Role
from ..store import Point, Route
from . import convert

ROLE_METADATA_KEY = "x-locomotion-role"
ROLES = {"upper-system": Role.UPPER_SYSTEM, "operator": Role.OPERATOR, "integrator": Role.INTEGRATOR}
WATCH_POSE_RATE = (1.0, 20.0, 5.0)  # [Hz] 下限, 上限, 既定


class StatusHub:
    """中核の状態通知を WatchStatus の各購読者に配る"""

    def __init__(self):
        self._lock = threading.Lock()
        self._subscribers: set[queue.Queue] = set()

    def publish(self, status):
        with self._lock:
            for q in self._subscribers:
                q.put(status)

    def subscribe(self):
        q = queue.Queue()
        with self._lock:
            self._subscribers.add(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            self._subscribers.discard(q)


def role_of(context) -> Role:
    value = dict(context.invocation_metadata()).get(ROLE_METADATA_KEY)
    if value is None:
        context.abort(grpc.StatusCode.UNAUTHENTICATED, f"metadata {ROLE_METADATA_KEY} がありません")
    if value not in ROLES:
        context.abort(grpc.StatusCode.UNAUTHENTICATED, f"不明な役割です: {value}")
    return ROLES[value]


def abort_rejected(context, error: Rejected):
    if isinstance(error, PreconditionFailed):
        detail = any_pb2.Any()
        detail.Pack(
            pb.RejectionDetail(
                current_state=convert.enum_to_pb(error.state),
                current_mode=convert.enum_to_pb(error.mode),
                reason=error.reason,
            )
        )
        status = status_pb2.Status(code=code_pb2.FAILED_PRECONDITION, message=error.reason, details=[detail])
        context.abort_with_status(rpc_status.to_status(status))
    code = {PermissionDenied: grpc.StatusCode.PERMISSION_DENIED, NotFound: grpc.StatusCode.NOT_FOUND}.get(
        type(error), grpc.StatusCode.INTERNAL
    )
    context.abort(code, str(error))


def call_core(context, function, *args, **kwargs):
    """中核の呼び出し。拒否は gRPC のステータスに変換する"""
    try:
        return function(*args, **kwargs)
    except Rejected as error:
        abort_rejected(context, error)


class ServerContext:
    """サービス間で共有するもの"""

    def __init__(self, core, adapter, store, hub):
        self.core = core
        self.adapter = adapter
        self.store = store
        self.hub = hub

    def map_version(self, map_id):
        return self.store.get_map(map_id).version if map_id and self.store.map_exists(map_id) else 0

    def status_pb(self, status=None):
        status = status or self.core.status()
        return convert.status_to_pb(status, self.adapter.localization(), self.map_version(status.active_map_id))


# ================================================================


class RobotService(pb_grpc.RobotServiceServicer):
    def __init__(self, ctx: ServerContext):
        self.ctx = ctx

    def GetStatus(self, request, context):
        role_of(context)
        return self.ctx.status_pb()

    def WatchStatus(self, request, context):
        role_of(context)
        q = self.ctx.hub.subscribe()
        try:
            last = self.ctx.core.status()
            yield self.ctx.status_pb(last)  # 接続直後に現在値を 1 回送る
            while context.is_active():
                try:
                    status = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                if status.sequence > last.sequence:
                    last = status
                    yield self.ctx.status_pb(status)
        finally:
            self.ctx.hub.unsubscribe(q)

    def WatchPose(self, request, context):
        role_of(context)
        low, high, default = WATCH_POSE_RATE
        rate = min(max(request.rate_hz or default, low), high)
        while context.is_active():
            robot = self.ctx.adapter.latest_pose()
            if robot is not None:
                yield pb.PoseUpdate(
                    stamp=convert.timestamp(robot.stamp),
                    map_id=self.ctx.core.active_map_id or "",
                    pose=convert.pose_to_pb(robot.pose),
                    linear_velocity_x=robot.linear_x,
                    linear_velocity_y=robot.linear_y,
                    angular_velocity=robot.angular,
                )
            time.sleep(1.0 / rate)

    def SetMode(self, request, context):
        role = role_of(context)
        mode = convert.enum_from_pb(model.Mode, request.mode)
        state = call_core(context, self.ctx.core.set_mode, role, mode)
        return pb.SetModeResponse(resulting_state=convert.enum_to_pb(state))

    def SetInitialPose(self, request, context):
        role = role_of(context)
        source = request.WhichOneof("source")
        if source == "pose":
            call_core(context, self.ctx.core.set_initial_pose, role, pose=convert.pose_from_pb(request.pose))
        elif source == "point_id":
            call_core(context, self.ctx.core.set_initial_pose, role, point_id=request.point_id)
        else:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "pose か point_id を指定してください")
        return pb.SetInitialPoseResponse()

    def Teleop(self, request, context):
        call_core(
            context,
            self.ctx.core.teleop,
            role_of(context),
            request.linear_velocity_x,
            request.linear_velocity_y,
            request.angular_velocity,
        )
        return pb.TeleopResponse()

    def ResetSafetyStop(self, request, context):
        call_core(context, self.ctx.core.reset_safety_stop, role_of(context))
        return pb.ResetSafetyStopResponse()


class TaskService(pb_grpc.TaskServiceServicer):
    def __init__(self, ctx: ServerContext):
        self.ctx = ctx

    def _task_pb(self, task, with_history=True):
        return convert.task_to_pb(task, self.ctx.map_version(task.map_id), with_history)

    def StartTask(self, request, context):
        role = role_of(context)
        point_ids = list(request.point_ids)
        if bool(point_ids) == bool(request.route_id):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "point_ids と route_id のどちらか一方を指定してください")
        if request.route_id:
            # 開始時点のルートの内容をタスクに記録する（仕様書 6 章「ルートのルール」）
            map_id = self.ctx.core.active_map_id
            if not map_id or not self.ctx.store.route_exists(map_id, request.route_id):
                context.abort(grpc.StatusCode.NOT_FOUND, f"ルート {request.route_id} は地図 {map_id} にありません")
            point_ids = list(self.ctx.store.get_route(map_id, request.route_id).point_ids)
        task = call_core(
            context,
            self.ctx.core.start_task,
            role,
            request.request_id,
            point_ids,
            request.expected_map_id,
            route_id=request.route_id,
        )
        return pb.StartTaskResponse(task=self._task_pb(task))

    def _command(self, method, request, context):
        task = call_core(context, method, role_of(context), request.task_id)
        return pb.TaskCommandResponse(task=self._task_pb(task))

    def PauseTask(self, request, context):
        return self._command(self.ctx.core.pause_task, request, context)

    def ResumeTask(self, request, context):
        return self._command(self.ctx.core.resume_task, request, context)

    def StopTask(self, request, context):
        return self._command(self.ctx.core.stop_task, request, context)

    def RetryTask(self, request, context):
        return self._command(self.ctx.core.retry_task, request, context)

    def GetTask(self, request, context):
        role_of(context)
        return self._task_pb(call_core(context, self.ctx.core.get_task, request.task_id))

    def ListTasks(self, request, context):
        role_of(context)
        start = convert.to_seconds(getattr(request, "from")) if request.HasField("from") else -math.inf
        end = convert.to_seconds(request.to) if request.HasField("to") else math.inf
        tasks = sorted(
            (t for t in self.ctx.core.tasks.values() if start <= t.created_at <= end), key=lambda t: t.created_at
        )
        offset = int(request.page_token or 0)
        size = request.page_size or 100
        page = tasks[offset : offset + size]
        next_token = str(offset + size) if offset + size < len(tasks) else ""
        return pb.ListTasksResponse(tasks=[self._task_pb(t, with_history=False) for t in page], next_page_token=next_token)


class MapService(pb_grpc.MapServiceServicer):
    """最小限: 地図の一覧・参照と使用する地図の切替（作成・編集はステップ 4）"""

    def __init__(self, ctx: ServerContext):
        self.ctx = ctx

    def _map_pb(self, map_id):
        info = self.ctx.store.get_map(map_id)
        return pb.MapInfo(map_id=info.map_id, name=info.name, version=info.version)

    def ListMaps(self, request, context):
        role_of(context)
        return pb.ListMapsResponse(maps=[self._map_pb(m) for m in self.ctx.store.map_ids()])

    def GetMap(self, request, context):
        role_of(context)
        if not self.ctx.store.map_exists(request.map_id):
            context.abort(grpc.StatusCode.NOT_FOUND, f"地図 {request.map_id} はありません")
        return self._map_pb(request.map_id)

    def ActivateMap(self, request, context):
        call_core(context, self.ctx.core.activate_map, role_of(context), request.map_id)
        self.ctx.store.set_active_map_id(request.map_id)
        return pb.ActivateMapResponse()


class PointService(pb_grpc.PointServiceServicer):
    """地点（最小限: 一覧・参照・登録。更新・削除・再確認はセットアップ機能のステップ）とルート"""

    def __init__(self, ctx: ServerContext):
        self.ctx = ctx

    def _require_map(self, context, map_id):
        if not self.ctx.store.map_exists(map_id):
            context.abort(grpc.StatusCode.NOT_FOUND, f"地図 {map_id} はありません")

    def ListPoints(self, request, context):
        role_of(context)
        self._require_map(context, request.map_id)
        return pb.ListPointsResponse(points=[convert.point_to_pb(p) for p in self.ctx.store.list_points(request.map_id)])

    def GetPoint(self, request, context):
        role_of(context)
        if not self.ctx.store.point_exists(request.map_id, request.point_id):
            context.abort(grpc.StatusCode.NOT_FOUND, f"地点 {request.point_id} はありません")
        return convert.point_to_pb(self.ctx.store.get_point(request.map_id, request.point_id))

    def CreatePoint(self, request, context):
        if role_of(context) != Role.INTEGRATOR:
            context.abort(grpc.StatusCode.PERMISSION_DENIED, "地点の登録は導入担当者のみです")
        self._require_map(context, request.map_id)
        if not request.point_id:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "point_id を指定してください")
        if self.ctx.store.point_exists(request.map_id, request.point_id):
            context.abort(grpc.StatusCode.ALREADY_EXISTS, f"地点 {request.point_id} は既にあります")

        if request.WhichOneof("pose_source") == "use_current_pose":
            # ティーチング: 現在位置を記録（自己位置が正常な場合のみ）
            status = self.ctx.core.status()
            robot = self.ctx.adapter.latest_pose()
            if not status.localized or robot is None or status.active_map_id != request.map_id:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "自己位置が確定していないため現在位置を記録できません")
            pose = robot.pose
        elif request.HasField("pose"):
            pose = convert.pose_from_pb(request.pose)
        else:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "pose か use_current_pose を指定してください")

        point = Point(
            map_id=request.map_id,
            point_id=request.point_id,
            name=request.name or request.point_id,
            pose=pose,
            kind=convert.point_kind_from_pb(request.kind),
            move_mode_id=request.move_mode_id,
            tolerance_xy=request.tolerance.xy,
            tolerance_yaw=request.tolerance.yaw,
        )
        self.ctx.store.put_point(point)
        return convert.point_to_pb(point)


    # ---- ルート ----
    def _require_integrator(self, context):
        if role_of(context) != Role.INTEGRATOR:
            context.abort(grpc.StatusCode.PERMISSION_DENIED, "ルートの登録・編集は導入担当者のみです")

    def _require_route(self, context, map_id, route_id, expected_version=None):
        self._require_map(context, map_id)
        if not self.ctx.store.route_exists(map_id, route_id):
            context.abort(grpc.StatusCode.NOT_FOUND, f"ルート {route_id} はありません")
        route = self.ctx.store.get_route(map_id, route_id)
        if expected_version is not None and expected_version != route.version:
            context.abort(grpc.StatusCode.ABORTED, f"ルート {route_id} は版 {route.version} に更新されています")
        return route

    def _validate_route_points(self, context, map_id, point_ids):
        if not point_ids:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "地点を 1 つ以上指定してください")
        missing = [p for p in point_ids if not self.ctx.store.point_exists(map_id, p)]
        if missing:
            context.abort(grpc.StatusCode.NOT_FOUND, f"地点 {', '.join(missing)} は地図 {map_id} にありません")

    def ListRoutes(self, request, context):
        role_of(context)
        self._require_map(context, request.map_id)
        return pb.ListRoutesResponse(routes=[convert.route_to_pb(r) for r in self.ctx.store.list_routes(request.map_id)])

    def GetRoute(self, request, context):
        role_of(context)
        return convert.route_to_pb(self._require_route(context, request.map_id, request.route_id))

    def CreateRoute(self, request, context):
        self._require_integrator(context)
        self._require_map(context, request.map_id)
        if not request.route_id:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "route_id を指定してください")
        if self.ctx.store.route_exists(request.map_id, request.route_id):
            context.abort(grpc.StatusCode.ALREADY_EXISTS, f"ルート {request.route_id} は既にあります")
        self._validate_route_points(context, request.map_id, list(request.point_ids))
        route = Route(request.map_id, request.route_id, request.name or request.route_id, list(request.point_ids))
        self.ctx.store.put_route(route)
        return convert.route_to_pb(route)

    def UpdateRoute(self, request, context):
        self._require_integrator(context)
        route = self._require_route(context, request.map_id, request.route_id, request.expected_version)
        self._validate_route_points(context, request.map_id, list(request.point_ids))
        updated = Route(
            request.map_id, request.route_id, request.name or route.name, list(request.point_ids), route.version + 1
        )
        self.ctx.store.put_route(updated)
        return convert.route_to_pb(updated)

    def DeleteRoute(self, request, context):
        self._require_integrator(context)
        self._require_route(context, request.map_id, request.route_id, request.expected_version)
        self.ctx.store.delete_route(request.map_id, request.route_id)
        return pb.DeleteRouteResponse()


def add_services(server, ctx: ServerContext):
    pb_grpc.add_RobotServiceServicer_to_server(RobotService(ctx), server)
    pb_grpc.add_TaskServiceServicer_to_server(TaskService(ctx), server)
    pb_grpc.add_MapServiceServicer_to_server(MapService(ctx), server)
    pb_grpc.add_PointServiceServicer_to_server(PointService(ctx), server)
