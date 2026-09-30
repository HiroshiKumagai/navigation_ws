"""ミッションサーバとの gRPC 接続。

- WatchStatus / WatchPose は別スレッドで受信し、Qt のシグナルで画面に渡す。切れたら自動で再接続する
- 指示系（unary）は call() で呼ぶ。拒否は ApiError（理由の文言つき）にする
"""

import threading
import time

import grpc
from grpc_status import rpc_status
from locomotion.v1 import locomotion_pb2 as pb
from locomotion.v1 import locomotion_pb2_grpc as pb_grpc
from PySide6.QtCore import QObject, Signal

from . import labels

CALL_TIMEOUT = 3.0  # [s]
RECONNECT_INTERVAL = 2.0  # [s]


class ApiError(Exception):
    """サーバが指示を受け付けなかった、または通信できなかった"""


class ApiClient(QObject):
    status_changed = Signal(object)  # pb.RobotStatus
    pose_changed = Signal(object)  # pb.PoseUpdate
    connection_changed = Signal(bool)

    def __init__(self, address, role):
        super().__init__()
        self.role = role
        self._metadata = [("x-locomotion-role", role)]
        self._channel = grpc.insecure_channel(address)
        self.robot = pb_grpc.RobotServiceStub(self._channel)
        self.task = pb_grpc.TaskServiceStub(self._channel)
        self.maps = pb_grpc.MapServiceStub(self._channel)
        self.points = pb_grpc.PointServiceStub(self._channel)
        self._running = True
        self._streams = []
        self.connected = False

    def start(self):
        for target in (self._watch_status, self._watch_pose):
            threading.Thread(target=target, daemon=True).start()

    def stop(self):
        self._running = False
        for stream in list(self._streams):
            stream.cancel()
        self._channel.close()

    # ---- 指示系 ----
    def call(self, method, request):
        try:
            return method(request, metadata=self._metadata, timeout=CALL_TIMEOUT)
        except grpc.RpcError as error:
            raise ApiError(self.describe(error)) from None

    @staticmethod
    def describe(error: grpc.RpcError):
        code = error.code()
        if code in (grpc.StatusCode.UNAVAILABLE, grpc.StatusCode.DEADLINE_EXCEEDED):
            return "ミッションサーバに接続できません"
        message = error.details() or code.name
        status = rpc_status.from_call(error)
        if status and status.details:
            detail = pb.RejectionDetail()
            if status.details[0].Unpack(detail):
                message = detail.reason or message
                message += f"（現在: {labels.STATE[detail.current_state][0]}）"
        if code == grpc.StatusCode.PERMISSION_DENIED:
            message = "この操作の権限がありません: " + message
        return message

    # ---- 配信の受信（別スレッド） ----
    def _set_connected(self, value):
        if value != self.connected:
            self.connected = value
            self.connection_changed.emit(value)

    def _watch_status(self):
        self._watch(lambda: self.robot.WatchStatus(pb.WatchStatusRequest(), metadata=self._metadata), self.status_changed)

    def _watch_pose(self):
        self._watch(lambda: self.robot.WatchPose(pb.WatchPoseRequest(rate_hz=10), metadata=self._metadata), self.pose_changed)

    def _watch(self, open_stream, signal):
        while self._running:
            stream = open_stream()
            self._streams.append(stream)
            try:
                for message in stream:
                    self._set_connected(True)
                    signal.emit(message)
            except grpc.RpcError as error:
                if error.code() != grpc.StatusCode.CANCELLED:
                    self._set_connected(False)
            finally:
                self._streams.remove(stream)
            if self._running:
                time.sleep(RECONNECT_INTERVAL)
