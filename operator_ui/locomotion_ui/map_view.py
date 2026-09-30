"""地図の表示（地図画像・地点・ロボット）と、地点のタップ・位置の指定。

座標: 地図の map フレーム（m）<-> 画像のピクセル。画像の 0 行目が y 最大、0 列目が x 最小（map_server 形式）。
"""

import math

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import QGraphicsScene, QGraphicsView

# 車体外形（base_link 基準、m）。ミッションサーバの設定から取れるようになるまでの仮置き
FOOTPRINT = [(0.98, 0.74), (0.98, -0.74), (-1.09, -0.74), (-1.09, 0.74)]
NOSE = [(0.98, 0.35), (1.45, 0.0), (0.98, -0.35)]

POINT_COLOR = QColor("#00897b")
POINT_SELECTED = QColor("#1565c0")
POINT_DONE = QColor("#9e9e9e")
POINT_FAILED = QColor("#c62828")
ROBOT_COLOR = QColor(30, 100, 220, 180)
TARGET_COLOR = QColor(200, 0, 160, 100)


class MapView(QGraphicsView):
    point_clicked = Signal(str)  # point_id
    pose_picked = Signal(object)  # (x, y, yaw) 位置の指定モードのとき

    def __init__(self):
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush(QColor("#cfd8dc"))
        self.resolution = 0.05
        self.origin = (0.0, 0.0)
        self.height_px = 0
        self._map_item = None
        self._points = {}  # point_id -> (Pose2D, [items])
        self._robot = self._polygon_item(ROBOT_COLOR, z=20)
        self._target = self._polygon_item(TARGET_COLOR, z=15, dashed=True)
        self._pick_mode = False
        self._press = None
        self._fitted = False

    # ---- 地図 ----
    def set_map(self, png: bytes, resolution, origin_x, origin_y):
        image = QImage.fromData(png, "PNG")
        if self._map_item is not None:
            self.scene().removeItem(self._map_item)
        self._map_item = self.scene().addPixmap(QPixmap.fromImage(image))
        self._map_item.setZValue(0)
        self.resolution, self.origin, self.height_px = resolution, (origin_x, origin_y), image.height()
        self.setSceneRect(self._map_item.boundingRect())
        self.fit()

    def to_scene(self, x, y):
        return QPointF((x - self.origin[0]) / self.resolution, self.height_px - (y - self.origin[1]) / self.resolution)

    def to_map(self, point: QPointF):
        return self.origin[0] + point.x() * self.resolution, self.origin[1] + (self.height_px - point.y()) * self.resolution

    # ---- 地点 ----
    def set_points(self, points):
        """points: pb.Point のリスト"""
        for _, items in self._points.values():
            for item in items:
                self.scene().removeItem(item)
        self._points = {}
        font = QFont()
        font.setPointSize(9)
        font.setBold(True)
        for point in points:
            center = self.to_scene(point.pose.x, point.pose.y)
            r = 0.45 / self.resolution
            diamond = QPolygonF([center + QPointF(0, -r), center + QPointF(r, 0), center + QPointF(0, r), center + QPointF(-r, 0)])
            shape = self.scene().addPolygon(diamond, QPen(Qt.white, 1.5), QBrush(POINT_COLOR))
            shape.setZValue(10)
            shape.setData(0, point.point_id)
            label = self.scene().addSimpleText(point.point_id, font)
            label.setBrush(QBrush(QColor("#263238")))
            label.setPos(center + QPointF(r * 1.2, -r * 1.4))
            label.setZValue(11)
            label.setData(0, point.point_id)
            self._points[point.point_id] = (point.pose, [shape, label])

    def color_points(self, colors: dict):
        """point_id -> QColor（指定のないものは既定の色）"""
        for point_id, (_, items) in self._points.items():
            items[0].setBrush(QBrush(colors.get(point_id, POINT_COLOR)))

    # ---- ロボット・位置の指定 ----
    def _polygon_item(self, color, z, dashed=False):
        pen = QPen(color.darker(160), 1.5)
        if dashed:
            pen.setStyle(Qt.DashLine)
        item = self.scene().addPolygon(QPolygonF(), pen, QBrush(color))
        item.setZValue(z)
        item.setVisible(False)
        return item

    def _footprint(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        # 車体の輪郭に、前方を示す三角形を挟む
        outline = [FOOTPRINT[0], *NOSE, FOOTPRINT[1], FOOTPRINT[2], FOOTPRINT[3]]
        return QPolygonF([self.to_scene(x + c * fx - s * fy, y + s * fx + c * fy) for fx, fy in outline])

    def set_robot(self, pose):
        """pose: (x, y, yaw) または None"""
        self._robot.setVisible(pose is not None and self._map_item is not None)
        if pose is not None:
            self._robot.setPolygon(self._footprint(*pose))

    def set_pick_mode(self, enabled):
        """位置の指定モード（初期位置の設定用）: タップで位置、ドラッグで向き"""
        self._pick_mode = enabled
        self.setDragMode(QGraphicsView.NoDrag if enabled else QGraphicsView.ScrollHandDrag)
        self.viewport().setCursor(Qt.CrossCursor if enabled else Qt.OpenHandCursor)
        if not enabled:
            self._target.setVisible(False)

    def show_target(self, pose):
        self._target.setVisible(pose is not None)
        if pose is not None:
            self._target.setPolygon(self._footprint(*pose))

    # ---- 表示範囲 ----
    def fit(self):
        if self._map_item is not None:
            self.fitInView(self._map_item, Qt.KeepAspectRatio)

    def zoom(self, factor):
        self.scale(factor, factor)

    def center_on_robot(self):
        if self._robot.isVisible():
            self.centerOn(self._robot.polygon().boundingRect().center())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._fitted and self._map_item is not None:
            self.fit()
            self._fitted = True

    def wheelEvent(self, event):
        self.zoom(1.15 if event.angleDelta().y() > 0 else 1 / 1.15)

    # ---- 入力 ----
    def mousePressEvent(self, event):
        scene_pos = self.mapToScene(event.position().toPoint())
        if self._pick_mode and event.button() == Qt.LeftButton:
            self._press = self.to_map(scene_pos)
            self._emit_pick(self._press, 0.0)
            return
        for item in self.scene().items(scene_pos):
            point_id = item.data(0)
            if point_id:
                self.point_clicked.emit(point_id)
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pick_mode and self._press is not None:
            x, y = self.to_map(self.mapToScene(event.position().toPoint()))
            if math.hypot(x - self._press[0], y - self._press[1]) > 0.3:
                self._emit_pick(self._press, math.atan2(y - self._press[1], x - self._press[0]))
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self._press = None
        super().mouseReleaseEvent(event)

    def _emit_pick(self, position, yaw):
        pose = (position[0], position[1], yaw)
        self.show_target(pose)
        self.pose_picked.emit(pose)
