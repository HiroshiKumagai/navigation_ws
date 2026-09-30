"""操作パッド（仕様書 8 章 Teleop）: 移動はスティックのドラッグ、旋回はボタンを押している間だけ。

- 操作中は約 10 Hz で速度指令を送る。指を離したら速度 0 を 1 回送って止める
- 通信が途切れても、サーバ側（指令切替ノード）が 0.5 秒で止める
"""

import math

from PySide6.QtCore import QPointF, Qt, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget

SEND_INTERVAL_MS = 100  # 10 Hz
MAX_LINEAR = 0.3  # [m/s] ゲームパッドの通常速度と同じ
MAX_ANGULAR = 0.4  # [rad/s]


class Stick(QWidget):
    """ドラッグした方向・距離に応じて (前後, 左右) を -1〜1 で返すスティック"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(170, 170)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.value = (0.0, 0.0)  # (forward, left)
        self._knob = QPointF(0, 0)
        self._dragging = False

    def _radius(self):
        return min(self.width(), self.height()) / 2 - 12

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2, self.height() / 2)
        r = self._radius()
        p.setPen(QPen(QColor("#90a4ae"), 2))
        p.setBrush(QBrush(QColor("#eceff1")))
        p.drawEllipse(center, r, r)
        p.setPen(QPen(QColor("#b0bec5"), 1, Qt.DashLine))
        p.drawLine(center + QPointF(-r, 0), center + QPointF(r, 0))
        p.drawLine(center + QPointF(0, -r), center + QPointF(0, r))
        p.setPen(QPen(QColor("#455a64")))
        for text, offset in (("前", (0, -r + 14)), ("後", (0, r - 6)), ("左", (-r + 12, 5)), ("右", (r - 12, 5))):
            p.drawText(center + QPointF(offset[0] - 6, offset[1]), text)
        p.setPen(QPen(QColor("#0d47a1"), 2))
        p.setBrush(QBrush(QColor("#1e88e5") if self._dragging else QColor("#90caf9")))
        p.drawEllipse(center + self._knob, r * 0.28, r * 0.28)

    def _update(self, pos):
        center = QPointF(self.width() / 2, self.height() / 2)
        d = pos - center
        r = self._radius()
        length = math.hypot(d.x(), d.y())
        if length > r:
            d = d * (r / length)
        self._knob = d
        self.value = (-d.y() / r, -d.x() / r)  # 画面の上 = 前進、左 = 左（+y）
        self.update()

    def mousePressEvent(self, event):
        self._dragging = True
        self._update(event.position())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update(event.position())

    def mouseReleaseEvent(self, event):
        self._dragging = False
        self._knob = QPointF(0, 0)
        self.value = (0.0, 0.0)
        self.update()

    @property
    def active(self):
        return self._dragging


class ControlPad(QWidget):
    velocity = Signal(float, float, float)  # (linear_x, linear_y, angular) を送るべきとき

    def __init__(self, note=""):
        super().__init__()
        self.stick = Stick()
        self.rotate_left = QPushButton("↺\n左旋回")
        self.rotate_right = QPushButton("↻\n右旋回")
        for button in (self.rotate_left, self.rotate_right):
            button.setMinimumSize(72, 72)
            button.setStyleSheet("font-size: 15px;")
            button.setAutoRepeat(False)

        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        stick_box = QVBoxLayout()
        stick_box.addWidget(QLabel("移動（ドラッグ）"))
        stick_box.addWidget(self.stick, stretch=1)
        rotate_box = QVBoxLayout()
        rotate_box.addWidget(QLabel("旋回（押している間）"))
        buttons = QHBoxLayout()
        buttons.addWidget(self.rotate_left)
        buttons.addWidget(self.rotate_right)
        rotate_box.addLayout(buttons)
        rotate_box.addStretch()
        row.addLayout(stick_box, stretch=3)
        row.addLayout(rotate_box, stretch=2)
        layout.addLayout(row)
        if note:
            hint = QLabel(note)
            hint.setWordWrap(True)
            hint.setStyleSheet("color: #607d8b;")
            layout.addWidget(hint)

        self._was_active = False
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(SEND_INTERVAL_MS)

    def _tick(self):
        if not self.isEnabled():
            return
        rotating = self.rotate_left.isDown() or self.rotate_right.isDown()
        active = self.stick.active or rotating
        if active:
            forward, left = self.stick.value
            angular = MAX_ANGULAR if self.rotate_left.isDown() else (-MAX_ANGULAR if self.rotate_right.isDown() else 0.0)
            self.velocity.emit(forward * MAX_LINEAR, left * MAX_LINEAR, angular)
        elif self._was_active:
            self.velocity.emit(0.0, 0.0, 0.0)  # 指を離したら 1 回だけ 0 を送る
        self._was_active = active
