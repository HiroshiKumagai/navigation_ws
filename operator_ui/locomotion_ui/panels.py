"""運用画面の右側のパネル: タスクの進捗、移動指示、モード、対応パネル（S2）。"""

import time

from locomotion.v1 import locomotion_pb2 as pb
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)

from . import labels

MARK_DONE, MARK_CURRENT, MARK_FAILED, MARK_TODO = "✓", "▶", "✕", "・"


def wrap_label(text="", style=""):
    label = QLabel(text)
    label.setWordWrap(True)
    if style:
        label.setStyleSheet(style)
    return label


class TaskProgressPanel(QGroupBox):
    """現在のタスク。巡回は地点ごとに ✓（到着済み）/ ▶（移動中）/ ✕（失敗）/ ・（未到着）"""

    def __init__(self):
        super().__init__("現在のタスク")
        layout = QVBoxLayout(self)
        self.title = wrap_label("—", "font-weight: bold;")
        self.lines = wrap_label("", "font-size: 15px;")
        self.lines.setTextFormat(Qt.RichText)
        self.footer = wrap_label("", "color: #607d8b;")
        layout.addWidget(self.title)
        layout.addWidget(self.lines)
        layout.addWidget(self.footer)
        self._created_at = None
        self._active = False

    def show_task(self, task, route_names, finished=False):
        if task is None:
            self.title.setText("タスクはありません")
            self.lines.setText("")
            self.footer.setText("")
            self._created_at = None
            return
        kind = "巡回" if len(task.point_ids) > 1 else "単発移動"
        route = f"「{route_names.get(task.route_id, task.route_id)}」" if task.route_id else ""
        status = labels.TASK_STATUS.get(task.status, "")
        self.title.setText(f"{kind}{route}　{status}" if not finished else f"前回のタスク: {kind}{route}　{status}")
        rows = []
        for i, point_id in enumerate(task.point_ids):
            if i < task.current_index or task.status == pb.TASK_STATUS_SUCCEEDED:
                mark, color = MARK_DONE, "#9e9e9e"
            elif i == task.current_index and task.status == pb.TASK_STATUS_FAULTED:
                mark, color = MARK_FAILED, "#c62828"
            elif i == task.current_index and task.status in (pb.TASK_STATUS_RUNNING, pb.TASK_STATUS_PAUSED):
                mark, color = MARK_CURRENT, "#1565c0"
            else:
                mark, color = MARK_TODO, "#263238"
            rows.append(f'<span style="color:{color}">{mark} {point_id}</span>')
        self.lines.setText("<br>".join(rows))
        self._created_at = task.created_at.ToSeconds() if not finished else None
        self._active = not finished
        self.tick()

    def tick(self):
        if self._created_at is None:
            self.footer.setText("")
            return
        elapsed = max(0, int(time.time() - self._created_at))
        self.footer.setText(f"経過 {elapsed // 60}:{elapsed % 60:02d}")


class CommandPanel(QGroupBox):
    """移動指示: ルートを選ぶ、または地点を順に選んで開始する"""

    start_points = Signal(list)
    start_route = Signal(str)

    def __init__(self):
        super().__init__("移動指示")
        self.destinations = []
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("ルート"))
        self.route_grid = QGridLayout()
        layout.addLayout(self.route_grid)
        layout.addWidget(QLabel("地点（押した順 / 地図の地点をタップ）"))
        self.point_grid = QGridLayout()
        layout.addLayout(self.point_grid)
        self.dest_label = wrap_label("行き先: なし", "font-size: 14px;")
        layout.addWidget(self.dest_label)
        row = QHBoxLayout()
        self.clear_button = QPushButton("クリア")
        self.clear_button.clicked.connect(self.clear)
        self.start_button = QPushButton("移動開始")
        self.start_button.setMinimumHeight(40)
        self.start_button.setStyleSheet("font-size: 15px; font-weight: bold;")
        self.start_button.clicked.connect(lambda: self.start_points.emit(list(self.destinations)))
        row.addWidget(self.clear_button)
        row.addWidget(self.start_button, stretch=2)
        layout.addLayout(row)
        self.reason = wrap_label("", "color: #c62828;")
        layout.addWidget(self.reason)
        self._enabled = True
        self._refresh()

    @staticmethod
    def _fill(grid, items, on_click, columns):
        while grid.count():
            grid.takeAt(0).widget().deleteLater()
        for i, (text, key) in enumerate(items):
            button = QPushButton(text)
            button.setMinimumHeight(34)
            button.clicked.connect(lambda _=False, k=key: on_click(k))
            grid.addWidget(button, i // columns, i % columns)

    def set_routes(self, routes):
        self._fill(self.route_grid, [(r.name or r.route_id, r.route_id) for r in routes], self._on_route, 2)

    def set_points(self, points):
        self._fill(self.point_grid, [(p.point_id, p.point_id) for p in points], self.add_destination, 3)

    def _on_route(self, route_id):
        if self._enabled:
            self.start_route.emit(route_id)

    def add_destination(self, point_id):
        if self._enabled:
            self.destinations.append(point_id)
            self._refresh()

    def clear(self):
        self.destinations = []
        self._refresh()

    def set_available(self, enabled, reason=""):
        self._enabled = enabled
        for grid in (self.route_grid, self.point_grid):
            for i in range(grid.count()):
                grid.itemAt(i).widget().setEnabled(enabled)
        self.clear_button.setEnabled(enabled)
        self.reason.setText(reason)
        self._refresh()

    def _refresh(self):
        text = " → ".join(self.destinations) if self.destinations else "なし"
        kind = "（巡回）" if len(self.destinations) > 1 else ""
        self.dest_label.setText(f"行き先: {text}{kind}")
        self.start_button.setEnabled(self._enabled and bool(self.destinations))


class ModePanel(QGroupBox):
    mode_requested = Signal(int)

    def __init__(self):
        super().__init__("モード")
        layout = QVBoxLayout(self)
        self.group = QButtonGroup(self)
        for mode in (pb.MODE_OPERATOR, pb.MODE_UPPER, pb.MODE_JOYSTICK):
            button = QRadioButton(labels.MODE[mode])
            self.group.addButton(button, mode)
            layout.addWidget(button)
        self.reason = wrap_label("", "color: #607d8b;")
        layout.addWidget(self.reason)
        self.group.idClicked.connect(self._on_clicked)
        self._current = None

    def _on_clicked(self, mode):
        if mode != self._current:
            self.mode_requested.emit(mode)
            self.set_mode(self._current)  # 実際の切替は状態の配信で反映する

    def set_mode(self, mode):
        self._current = mode
        button = self.group.button(mode) if mode else None
        if button is not None:
            button.setChecked(True)

    def set_available(self, enabled, reason=""):
        for button in self.group.buttons():
            button.setEnabled(enabled)
        self.reason.setText(reason)


class ResponsePanel(QGroupBox):
    """対応パネル（S2）: 準備未完了・異常停止・安全停止のときに、何が起きたか・何をするか・押すボタンを出す"""

    set_pose = Signal(object)  # (x, y, yaw)
    set_pose_point = Signal(str)
    retry = Signal()
    stop = Signal()
    to_manual = Signal()
    reset_safety = Signal()

    def __init__(self):
        super().__init__()
        self.setStyleSheet("QGroupBox { border: 2px solid #c62828; border-radius: 6px; margin-top: 8px; }")
        layout = QVBoxLayout(self)
        self.heading = wrap_label("", "font-size: 18px; font-weight: bold; color: #c62828;")
        self.what = wrap_label()
        self.action = wrap_label()
        layout.addWidget(self.heading)
        layout.addWidget(wrap_label("何が起きたか", "color: #607d8b;"))
        layout.addWidget(self.what)
        layout.addWidget(wrap_label("現場で何をするか", "color: #607d8b;"))
        layout.addWidget(self.action)

        # 位置の指定（準備未完了・自己位置喪失）
        self.pose_box = QWidget()
        pose_layout = QVBoxLayout(self.pose_box)
        pose_layout.setContentsMargins(0, 0, 0, 0)
        self.picked_label = wrap_label("地図上の位置: 未指定")
        self.set_pose_button = QPushButton("この位置に設定")
        self.set_pose_button.setEnabled(False)
        self.set_pose_button.clicked.connect(lambda: self.set_pose.emit(self._picked))
        row = QHBoxLayout()
        self.point_combo = QComboBox()
        point_button = QPushButton("地点で設定")
        point_button.clicked.connect(lambda: self.set_pose_point.emit(self.point_combo.currentData()))
        row.addWidget(self.point_combo, stretch=1)
        row.addWidget(point_button)
        pose_layout.addWidget(self.picked_label)
        pose_layout.addWidget(self.set_pose_button)
        pose_layout.addLayout(row)
        layout.addWidget(self.pose_box)

        self.buttons = {}
        for key, text, signal in (
            ("retry", "再試行", self.retry),
            ("manual", "手動操作に切り替え", self.to_manual),
            ("reset", "リセット", self.reset_safety),
            ("stop", "タスクを中止", self.stop),
        ):
            button = QPushButton(text)
            button.setMinimumHeight(38)
            button.clicked.connect(signal.emit)
            layout.addWidget(button)
            self.buttons[key] = button
        self.detail = wrap_label("", "color: #9e9e9e; font-size: 11px;")
        layout.addWidget(self.detail)
        layout.addStretch()
        self._picked = None

    def set_points(self, points):
        self.point_combo.clear()
        for p in points:
            self.point_combo.addItem(f"{p.point_id}　{p.name}", p.point_id)

    def picked(self, pose):
        self._picked = pose
        self.picked_label.setText(f"地図上の位置: ({pose[0]:.2f}, {pose[1]:.2f})")
        self.set_pose_button.setEnabled(True)

    def show_status(self, status):
        """表示内容を状態に合わせる。位置の指定が必要なら True を返す"""
        visible = {"retry": False, "manual": False, "reset": False, "stop": False}
        needs_pose = False
        detail = ""
        if status.state == pb.ROBOT_STATE_NOT_READY:
            heading, what, action = labels.NOT_READY
            needs_pose = True
            visible["manual"] = True
        elif status.state == pb.ROBOT_STATE_FAULT:
            heading, what, action = labels.FAULT[status.fault.category]
            heading = f"異常停止: {heading}"
            category = status.fault.category
            visible["retry"] = category in (pb.FAULT_CATEGORY_PATH_BLOCKED, pb.FAULT_CATEGORY_LOCALIZATION_LOST)
            visible["manual"] = category == pb.FAULT_CATEGORY_LOCALIZATION_LOST
            visible["stop"] = True
            needs_pose = category == pb.FAULT_CATEGORY_LOCALIZATION_LOST
            detail = f"詳細: {status.fault.detail_code} {status.fault.detail_message}".strip()
        else:  # 安全停止
            heading, what, action = labels.SAFETY[status.safety_stop_kind]
            heading = f"安全停止: {heading}"
            visible["reset"] = status.safety_stop_kind == pb.SAFETY_STOP_KIND_EMERGENCY
        self.heading.setText(f"⚠ {heading}")
        self.what.setText(what)
        self.action.setText(action)
        self.detail.setText(detail)
        self.pose_box.setVisible(needs_pose)
        for key, button in self.buttons.items():
            button.setVisible(visible[key])
        if not needs_pose:
            self._picked = None
            self.set_pose_button.setEnabled(False)
            self.picked_label.setText("地図上の位置: 未指定")
        return needs_pose
