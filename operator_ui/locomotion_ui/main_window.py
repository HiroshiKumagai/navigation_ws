"""メインウィンドウ: 共通ヘッダ（S0）、運用画面（S1 + S2）、実行記録（S3）。

使い方: .venv/bin/python -m locomotion_ui [--server localhost:50051] [--role operator|integrator]
"""

import argparse
import sys
import uuid
from datetime import timezone

from locomotion.v1 import locomotion_pb2 as pb
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMenu,
    QPushButton,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from . import labels
from .api import ApiClient, ApiError
from .control_pad import ControlPad
from .map_view import POINT_DONE, POINT_FAILED, POINT_SELECTED, MapView
from .panels import CommandPanel, ModePanel, ResponsePanel, TaskProgressPanel, wrap_label

ROLES = {"オペレータ": "operator", "導入担当者": "integrator"}
MODE_SWITCHABLE = {pb.ROBOT_STATE_NOT_READY, pb.ROBOT_STATE_IDLE, pb.ROBOT_STATE_FAULT, pb.ROBOT_STATE_MANUAL}
RESPONSE_STATES = {pb.ROBOT_STATE_NOT_READY, pb.ROBOT_STATE_FAULT, pb.ROBOT_STATE_SAFETY_STOP}
MANUAL_STATES = {pb.ROBOT_STATE_MANUAL, pb.ROBOT_STATE_MAPPING}


def chip(text="", color="#607d8b"):
    label = QLabel(text)
    label.setAlignment(Qt.AlignCenter)
    label.setStyleSheet(f"background:{color}; color:white; border-radius:4px; padding:4px 10px; font-weight:bold;")
    return label


class Header(QWidget):
    """共通ヘッダ（S0）: 状態・モード・安全系・地図・接続と、常に同じ場所の一時停止（再開）・停止"""

    def __init__(self):
        super().__init__()
        self.setStyleSheet("Header { background: #263238; }")
        self.setAttribute(Qt.WA_StyledBackground, True)
        layout = QHBoxLayout(self)
        self.state = chip("—")
        self.state.setStyleSheet(self.state.styleSheet() + "font-size: 16px;")
        self.mode = QLabel("—")
        self.safety = QLabel("—")
        self.map = QLabel("—")
        self.connection = QLabel("—")
        for widget in (self.mode, self.safety, self.map, self.connection):
            widget.setStyleSheet("color: #eceff1; padding: 0 8px;")
        self.pause = QPushButton("一時停止")
        self.stop = QPushButton("停止")
        for button, color in ((self.pause, "#f9a825"), (self.stop, "#c62828")):
            button.setMinimumSize(110, 40)
            button.setStyleSheet(
                f"QPushButton {{ background:{color}; color:white; font-size:16px; font-weight:bold; border-radius:6px; }}"
                "QPushButton:disabled { background:#546e7a; color:#90a4ae; }"
            )
        layout.addWidget(self.state)
        for widget in (self.mode, self.safety, self.map, self.connection):
            layout.addWidget(widget)
        layout.addStretch()
        layout.addWidget(self.pause)
        layout.addWidget(self.stop)

    def show_status(self, status, map_name):
        text, color = labels.STATE[status.state]
        if status.state == pb.ROBOT_STATE_NAVIGATING:
            text += f"（{labels.NAV_ACTIVITY.get(status.nav_activity, '')}）"
        elif status.state == pb.ROBOT_STATE_SAFETY_STOP:
            text += f"（{labels.SAFETY_KIND.get(status.safety_stop_kind, '')}）"
        elif status.state == pb.ROBOT_STATE_FAULT:
            text += f"（{labels.FAULT[status.fault.category][0]}）"
        self.state.setText(f"● {text}")
        self.state.setStyleSheet(
            f"background:{color}; color:white; border-radius:4px; padding:4px 10px; font-weight:bold; font-size:16px;"
        )
        self.mode.setText(labels.MODE.get(status.mode, "—"))
        safety = labels.SAFETY_KIND.get(status.safety_stop_kind) if status.state == pb.ROBOT_STATE_SAFETY_STOP else "正常"
        self.safety.setText(f"安全系: {safety}")
        self.map.setText(f"地図: {map_name} v{status.active_map_version}" if status.active_map_id else "地図: 未選択")
        self.pause.setText("再開" if status.state == pb.ROBOT_STATE_PAUSED else "一時停止")
        self.pause.setEnabled(status.state in (pb.ROBOT_STATE_NAVIGATING, pb.ROBOT_STATE_PAUSED))
        self.stop.setEnabled(status.state in (pb.ROBOT_STATE_NAVIGATING, pb.ROBOT_STATE_PAUSED, pb.ROBOT_STATE_FAULT))

    def show_connection(self, connected):
        self.connection.setText("● 接続中" if connected else "● 未接続")
        self.connection.setStyleSheet(f"color: {'#81c784' if connected else '#ef9a9a'}; padding: 0 8px;")


class HistoryPage(QWidget):
    """実行記録（S3）"""

    COLUMNS = ["開始", "指示元", "結果", "行き先", "進捗", "異常"]

    def __init__(self, api):
        super().__init__()
        self.api = api
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel("実行記録（新しい順）"))
        top.addStretch()
        refresh = QPushButton("更新")
        refresh.clicked.connect(self.refresh)
        top.addWidget(refresh)
        layout.addLayout(top)
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self._show_detail)
        layout.addWidget(self.table, stretch=2)
        self.detail = QTextEdit()
        self.detail.setReadOnly(True)
        layout.addWidget(self.detail, stretch=1)
        self.tasks = []

    def refresh(self):
        try:
            tasks = self.api.call(self.api.task.ListTasks, pb.ListTasksRequest()).tasks
        except ApiError as error:
            self.detail.setText(str(error))
            return
        self.tasks = list(reversed(tasks))
        self.table.setRowCount(len(self.tasks))
        for row, t in enumerate(self.tasks):
            route = f"「{t.route_id}」 " if t.route_id else ""
            cells = [
                t.created_at.ToDatetime(tzinfo=timezone.utc).astimezone().strftime("%m/%d %H:%M:%S"),
                labels.TASK_SOURCE.get(t.source, ""),
                labels.TASK_STATUS.get(t.status, ""),
                route + " → ".join(t.point_ids),
                f"{t.current_index}/{len(t.point_ids)}",
                "",
            ]
            for col, text in enumerate(cells):
                self.table.setItem(row, col, QTableWidgetItem(text))
        self.table.resizeColumnsToContents()

    def _show_detail(self):
        rows = {i.row() for i in self.table.selectedItems()}
        if not rows:
            return
        task_id = self.tasks[rows.pop()].task_id
        try:
            task = self.api.call(self.api.task.GetTask, pb.GetTaskRequest(task_id=task_id))
        except ApiError as error:
            self.detail.setText(str(error))
            return
        lines = [f"タスク {task.task_id}"]
        for e in task.history:
            fault = ""
            if e.HasField("fault"):
                fault = f"  {labels.FAULT[e.fault.category][0]}（{e.fault.detail_code}）"
            lines.append(
                f"{e.at.ToDatetime(tzinfo=timezone.utc).astimezone():%H:%M:%S}  {labels.TASK_STATUS.get(e.status, ''):6s}"
                f"  ロボット: {labels.STATE[e.robot_state][0]}{fault}"
            )
        self.detail.setText("\n".join(lines))
        faults = [e for e in task.history if e.HasField("fault")]
        if faults:
            self.table.item(self.table.currentRow(), 5).setText(labels.FAULT[faults[-1].fault.category][0])


class MainWindow(QMainWindow):
    def __init__(self, api: ApiClient):
        super().__init__()
        self.api = api
        self.setWindowTitle(f"ロコモーション 運用（{'導入担当者' if api.role == 'integrator' else 'オペレータ'}）")
        self.status = None
        self.map_id = None
        self.map_name = ""
        self.points = []
        self.routes = []
        self.last_task_id = None
        self.last_finished_task = None

        # ---- 運用画面 ----
        self.header = Header()
        self.map_view = MapView()
        map_tools = QHBoxLayout()
        for text, handler in (
            ("＋", lambda: self.map_view.zoom(1.25)),
            ("－", lambda: self.map_view.zoom(0.8)),
            ("全体", self.map_view.fit),
            ("ロボットに合わせる", self.map_view.center_on_robot),
        ):
            button = QPushButton(text)
            button.clicked.connect(handler)
            map_tools.addWidget(button)
        map_tools.addStretch()
        self.map_hint = QLabel("")
        map_tools.addWidget(self.map_hint)
        map_column = QVBoxLayout()
        map_column.addWidget(self.map_view, stretch=1)
        map_column.addLayout(map_tools)

        self.progress = TaskProgressPanel()
        self.command = CommandPanel()
        self.upper_note = wrap_label(labels.UPPER_MODE_NOTE, "font-size: 14px; padding: 12px;")
        self.control_pad = ControlPad(labels.JOYSTICK_NOTE)
        self.response = ResponsePanel()
        self.mode_panel = ModePanel()

        normal = QWidget()
        normal_layout = QVBoxLayout(normal)
        normal_layout.setContentsMargins(0, 0, 0, 0)
        normal_layout.addWidget(self.progress)
        normal_layout.addWidget(self.command)
        normal_layout.addWidget(self.upper_note)
        normal_layout.addStretch()
        manual = QWidget()
        manual_layout = QVBoxLayout(manual)
        manual_layout.setContentsMargins(0, 0, 0, 0)
        manual_title = wrap_label("手動操作", "font-size: 16px; font-weight: bold;")
        manual_layout.addWidget(manual_title)
        manual_layout.addWidget(self.control_pad, stretch=1)
        self.right = QStackedWidget()
        for page in (normal, manual, self.response):
            self.right.addWidget(page)
        self.page_normal, self.page_manual, self.page_response = normal, manual, self.response

        right_column = QVBoxLayout()
        right_column.addWidget(self.right, stretch=1)
        right_column.addWidget(self.mode_panel)
        right_widget = QWidget()
        right_widget.setLayout(right_column)
        right_widget.setFixedWidth(360)

        operation = QWidget()
        body = QHBoxLayout(operation)
        body.addLayout(map_column, stretch=1)
        body.addWidget(right_widget)

        # ---- 画面の切替（下部） ----
        self.history = HistoryPage(api)
        self.pages = QStackedWidget()
        self.pages.addWidget(operation)
        self.pages.addWidget(self.history)
        nav = QHBoxLayout()
        for text, index in (("運用", 0), ("実行記録", 1)):
            button = QPushButton(text)
            button.setMinimumHeight(34)
            button.clicked.connect(lambda _=False, i=index: self._show_page(i))
            nav.addWidget(button)
        nav.addStretch()
        if api.role == "integrator":
            setup = QPushButton("セットアップ ▼")
            menu = QMenu(self)
            for text in ("地図の作成（ステップ 3）", "地図の編集", "地点・ルートの登録", "テスト", "地図・地点の更新", "設定"):
                menu.addAction(text).setEnabled(False)
            setup.setMenu(menu)
            nav.addWidget(setup)

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.addWidget(self.header)
        root_layout.addWidget(self.pages, stretch=1)
        nav_widget = QWidget()
        nav_widget.setLayout(nav)
        root_layout.addWidget(nav_widget)
        self.setCentralWidget(root)
        self.statusBar().showMessage("接続しています…")

        # ---- 配線 ----
        api.status_changed.connect(self.on_status)
        api.pose_changed.connect(self.on_pose)
        api.connection_changed.connect(self.on_connection)
        self.header.pause.clicked.connect(self.on_pause_resume)
        self.header.stop.clicked.connect(lambda: self._task_command(api.task.StopTask, "停止しました"))
        self.map_view.point_clicked.connect(self.command.add_destination)
        self.map_view.pose_picked.connect(self.response.picked)
        self.command.start_points.connect(self.start_points)
        self.command.start_route.connect(self.start_route)
        self.mode_panel.mode_requested.connect(self.set_mode)
        self.control_pad.velocity.connect(self.teleop)
        self.response.set_pose.connect(self.set_initial_pose)
        self.response.set_pose_point.connect(lambda point_id: self.set_initial_pose(None, point_id))
        self.response.retry.connect(lambda: self._task_command(api.task.RetryTask, "再試行しました"))
        self.response.stop.connect(lambda: self._task_command(api.task.StopTask, "タスクを中止しました"))
        self.response.to_manual.connect(lambda: self.set_mode(pb.MODE_JOYSTICK))
        self.response.reset_safety.connect(self.reset_safety)
        self.header.show_connection(False)

        self._clock = QTimer(self)
        self._clock.timeout.connect(self.progress.tick)
        self._clock.start(1000)

    # ================================================================
    # 受信
    # ================================================================

    def on_connection(self, connected):
        self.header.show_connection(connected)
        self.statusBar().showMessage("接続しました" if connected else "ミッションサーバに接続できません（再接続します）", 5000)

    def on_status(self, status):
        self.status = status
        if status.active_map_id and status.active_map_id != self.map_id:
            self._load_map(status.active_map_id)
        self.header.show_status(status, self.map_name)

        # タスク: 保持中のものを表示。終わったら結果を取りにいく
        if status.HasField("active_task"):
            self.last_task_id = status.active_task.task_id
            self.last_finished_task = None
            self.progress.show_task(status.active_task, self._route_names())
        else:
            if self.last_task_id and self.last_finished_task is None:
                try:
                    self.last_finished_task = self.api.call(
                        self.api.task.GetTask, pb.GetTaskRequest(task_id=self.last_task_id)
                    )
                except ApiError:
                    pass
            self.progress.show_task(self.last_finished_task, self._route_names(), finished=True)
        self._color_points(status)

        # 右側: 状態に応じて切り替える
        if status.state in RESPONSE_STATES:
            self.right.setCurrentWidget(self.page_response)
            needs_pose = self.response.show_status(status)
            self.map_view.set_pick_mode(needs_pose)
            self.map_hint.setText("地図をタップしてロボットの位置、ドラッグで向きを指定" if needs_pose else "")
        else:
            self.map_view.set_pick_mode(False)
            self.map_hint.setText("地点をタップすると行き先に追加")
            if status.state in MANUAL_STATES:
                self.right.setCurrentWidget(self.page_manual)
            else:
                self.right.setCurrentWidget(self.page_normal)
        upper = status.mode == pb.MODE_UPPER
        self.command.setVisible(not upper)
        self.upper_note.setVisible(upper)
        idle = status.state == pb.ROBOT_STATE_IDLE
        self.command.set_available(idle, "" if idle else "待機中のみ指示できます")
        self.control_pad.setEnabled(status.joystick_accepting)

        self.mode_panel.set_mode(status.mode)
        switchable = status.state in MODE_SWITCHABLE
        state_name = labels.STATE[status.state][0]
        self.mode_panel.set_available(switchable, "" if switchable else f"{state_name}中はモードを切り替えられません")

    def on_pose(self, update):
        self.map_view.set_robot((update.pose.x, update.pose.y, update.pose.yaw))

    # ================================================================
    # 操作
    # ================================================================

    def _call(self, method, request, done_message=""):
        try:
            response = self.api.call(method, request)
        except ApiError as error:
            self.statusBar().showMessage(f"受け付けられませんでした: {error}", 8000)
            return None
        if done_message:
            self.statusBar().showMessage(done_message, 4000)
        return response

    def _task_command(self, method, done_message):
        if self.status is None or not self.status.HasField("active_task"):
            return
        self._call(method, pb.TaskCommandRequest(task_id=self.status.active_task.task_id), done_message)

    def on_pause_resume(self):
        if self.status is None:
            return
        if self.status.state == pb.ROBOT_STATE_PAUSED:
            self._task_command(self.api.task.ResumeTask, "再開しました")
        else:
            self._task_command(self.api.task.PauseTask, "一時停止しました")

    def start_points(self, point_ids):
        request = pb.StartTaskRequest(request_id=uuid.uuid4().hex, point_ids=point_ids)
        if self._call(self.api.task.StartTask, request, "移動を開始しました"):
            self.command.clear()

    def start_route(self, route_id):
        request = pb.StartTaskRequest(request_id=uuid.uuid4().hex, route_id=route_id)
        self._call(self.api.task.StartTask, request, f"ルート「{self._route_names().get(route_id, route_id)}」を開始しました")

    def set_mode(self, mode):
        self._call(self.api.robot.SetMode, pb.SetModeRequest(mode=mode), f"{labels.MODE[mode]}に切り替えました")

    def set_initial_pose(self, pose, point_id=None):
        if point_id:
            request = pb.SetInitialPoseRequest(point_id=point_id)
        else:
            x, y, yaw = pose
            request = pb.SetInitialPoseRequest(pose=pb.Pose2D(x=x, y=y, yaw=yaw))
        self._call(self.api.robot.SetInitialPose, request, "位置を設定しました")

    def reset_safety(self):
        self._call(self.api.robot.ResetSafetyStop, pb.ResetSafetyStopRequest(), "リセットを要求しました")

    def teleop(self, linear_x, linear_y, angular):
        try:
            self.api.call(
                self.api.robot.Teleop,
                pb.TeleopRequest(linear_velocity_x=linear_x, linear_velocity_y=linear_y, angular_velocity=angular),
            )
        except ApiError as error:
            self.statusBar().showMessage(f"手動操作: {error}", 3000)

    # ================================================================
    # 内部
    # ================================================================

    def _show_page(self, index):
        self.pages.setCurrentIndex(index)
        if index == 1:
            self.history.refresh()

    def _route_names(self):
        return {r.route_id: r.name for r in self.routes}

    def _load_map(self, map_id):
        try:
            info = self.api.call(self.api.maps.GetMap, pb.GetMapRequest(map_id=map_id))
            chunks = self.api.maps.DownloadMapImage(pb.DownloadMapImageRequest(map_id=map_id), metadata=self.api._metadata)
            png = b"".join(chunk.data for chunk in chunks)
            self.points = list(self.api.call(self.api.points.ListPoints, pb.ListPointsRequest(map_id=map_id)).points)
            self.routes = list(self.api.call(self.api.points.ListRoutes, pb.ListRoutesRequest(map_id=map_id)).routes)
        except Exception as error:  # 次の状態通知で再試行する
            self.statusBar().showMessage(f"地図を読み込めません: {error}", 8000)
            return
        self.map_id, self.map_name = map_id, info.name
        self.map_view.set_map(png, info.resolution, info.origin.x, info.origin.y)
        self.map_view.set_points(self.points)
        self.command.set_points(self.points)
        self.command.set_routes(self.routes)
        self.response.set_points(self.points)

    def _color_points(self, status):
        colors = {}
        task = status.active_task if status.HasField("active_task") else None
        for point_id in self.command.destinations:
            colors[point_id] = POINT_SELECTED
        if task is not None:
            for i, point_id in enumerate(task.point_ids):
                if i < task.current_index:
                    colors[point_id] = POINT_DONE
                elif i == task.current_index:
                    colors[point_id] = POINT_FAILED if task.status == pb.TASK_STATUS_FAULTED else POINT_SELECTED
        self.map_view.color_points(colors)

    def closeEvent(self, event):
        self.api.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--role", choices=list(ROLES.values()), help="省略時は起動時に選ぶ")
    args, qt_args = parser.parse_known_args()

    app = QApplication([sys.argv[0], *qt_args])
    role = args.role
    if role is None:
        # 本番の認証ができるまでの仮の役割選択
        name, ok = QInputDialog.getItem(None, "ロコモーション", "役割を選んでください", list(ROLES), 0, False)
        if not ok:
            return
        role = ROLES[name]
    api = ApiClient(args.server, role)
    window = MainWindow(api)
    window.resize(1280, 800)
    window.show()
    api.start()
    app.exec()


if __name__ == "__main__":
    main()
