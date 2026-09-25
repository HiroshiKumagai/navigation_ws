"""地図・地点・ロボット設定の保存（仕様書 6 章）。最小構成: YAML ファイルで持つ。

    <data_dir>/settings.yaml                 ロボット設定（使用中の地図 ID など）
    <data_dir>/maps/<map_id>/map.yaml        地図本体（map_server 形式。画像は同じディレクトリ）
    <data_dir>/maps/<map_id>/meta.yaml       名称・版番号
    <data_dir>/maps/<map_id>/points.yaml     目標地点
"""

import math
import threading
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .core.model import Pose2D


@dataclass
class Point:
    map_id: str
    point_id: str
    name: str
    pose: Pose2D
    kind: str = "WORK"  # WORK / STANDBY / CHARGE
    move_mode_id: str = ""
    tolerance_xy: float = 0.0  # 0 は Nav2 の既定値
    tolerance_yaw: float = 0.0
    needs_recheck: bool = False
    version: int = 1

    def to_yaml(self):
        data = {
            "point_id": self.point_id,
            "name": self.name,
            "pose": {"x": self.pose.x, "y": self.pose.y, "yaw_deg": round(math.degrees(self.pose.yaw), 3)},
            "kind": self.kind,
        }
        if self.move_mode_id:
            data["move_mode_id"] = self.move_mode_id
        if self.tolerance_xy or self.tolerance_yaw:
            data["tolerance"] = {"xy": self.tolerance_xy, "yaw": self.tolerance_yaw}
        if self.needs_recheck:
            data["needs_recheck"] = True
        if self.version != 1:
            data["version"] = self.version
        return data

    @classmethod
    def from_yaml(cls, map_id, data):
        pose = data["pose"]
        tolerance = data.get("tolerance", {})
        return cls(
            map_id=map_id,
            point_id=str(data["point_id"]),
            name=data.get("name", ""),
            pose=Pose2D(float(pose["x"]), float(pose["y"]), math.radians(float(pose.get("yaw_deg", 0.0)))),
            kind=data.get("kind", "WORK"),
            move_mode_id=data.get("move_mode_id", ""),
            tolerance_xy=float(tolerance.get("xy", 0.0)),
            tolerance_yaw=float(tolerance.get("yaw", 0.0)),
            needs_recheck=bool(data.get("needs_recheck", False)),
            version=int(data.get("version", 1)),
        )


@dataclass
class MapInfo:
    map_id: str
    name: str
    version: int
    yaml_path: Path
    points: dict[str, Point] = field(default_factory=dict)


class Store:
    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
        self._lock = threading.RLock()
        self._maps: dict[str, MapInfo] = {}
        self.settings: dict = {}
        self.reload()

    def reload(self):
        with self._lock:
            self.settings = self._read_yaml(self.data_dir / "settings.yaml", {})
            self._maps = {}
            maps_dir = self.data_dir / "maps"
            for map_dir in sorted(p for p in maps_dir.glob("*") if (p / "map.yaml").exists()):
                meta = self._read_yaml(map_dir / "meta.yaml", {})
                info = MapInfo(
                    map_id=map_dir.name,
                    name=meta.get("name", map_dir.name),
                    version=int(meta.get("version", 1)),
                    yaml_path=map_dir / "map.yaml",
                )
                for data in self._read_yaml(map_dir / "points.yaml", []) or []:
                    point = Point.from_yaml(info.map_id, data)
                    info.points[point.point_id] = point
                self._maps[info.map_id] = info

    # ---- 地図 ----
    def map_ids(self):
        with self._lock:
            return list(self._maps)

    def map_exists(self, map_id):
        with self._lock:
            return map_id in self._maps

    def get_map(self, map_id) -> MapInfo:
        with self._lock:
            return self._maps[map_id]

    # ---- 地点 ----
    def point_exists(self, map_id, point_id):
        with self._lock:
            return map_id in self._maps and point_id in self._maps[map_id].points

    def get_point(self, map_id, point_id) -> Point:
        with self._lock:
            return self._maps[map_id].points[point_id]

    def list_points(self, map_id) -> list[Point]:
        with self._lock:
            return list(self._maps[map_id].points.values())

    def put_point(self, point: Point):
        with self._lock:
            self._maps[point.map_id].points[point.point_id] = point
            self._save_points(point.map_id)

    def delete_point(self, map_id, point_id):
        with self._lock:
            del self._maps[map_id].points[point_id]
            self._save_points(map_id)

    # ---- ロボット設定 ----
    @property
    def active_map_id(self):
        return self.settings.get("active_map_id")

    def set_active_map_id(self, map_id):
        with self._lock:
            self.settings["active_map_id"] = map_id
            self._write_yaml(self.data_dir / "settings.yaml", self.settings)

    # ---- 内部 ----
    def _save_points(self, map_id):
        info = self._maps[map_id]
        self._write_yaml(info.yaml_path.parent / "points.yaml", [p.to_yaml() for p in info.points.values()])

    @staticmethod
    def _read_yaml(path, default):
        if not path.exists():
            return default
        return yaml.safe_load(path.read_text()) or default

    @staticmethod
    def _write_yaml(path, data):
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
        tmp.replace(path)
