"""Isaac Sim の環境 USD から、Nav2 の map_server 用の占有格子地図（PGM + YAML）を生成する。

衝突形状（PhysX コライダ）を LiDAR 高さの帯で切り取るため、見た目ではなく当たり判定どおりの地図になる。
座標系は Isaac Sim のワールド座標をそのまま map フレームとする。

使い方:
    ~/isaacsim/python.sh sim/tools/generate_map.py --output src/o3dyn_navigation/maps/warehouse
"""

import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--env", default="/Isaac/Environments/Simple_Warehouse/warehouse.usd")
parser.add_argument("--output", required=True, help="出力パス（拡張子なし）。<output>.pgm と <output>.yaml を作る")
parser.add_argument("--resolution", type=float, default=0.05, help="[m/pixel]")
parser.add_argument("--z-min", type=float, default=0.1, help="障害物として扱う高さの下限 [m]")
parser.add_argument("--z-max", type=float, default=0.6, help="障害物として扱う高さの上限 [m]")
parser.add_argument("--origin", type=float, nargs=2, default=[-6.0, -1.0], help="空いている地点 [m]（ロボットの初期位置）")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

app_utils.enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()
from isaacsim.asset.gen.omap.bindings import _omap  # noqa: E402

# シミュレーション（o3dyn_bringup.py）と同じく、新しいステージに /Environment として参照で読み込む。
# 環境 USD を直接開くと、参照で読み込んだ場合と位置がずれる（warehouse.usd では x 方向に 0.25 m）
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402

omni.usd.get_context().new_stage()
simulation_app.update()
stage_utils.add_reference_to_stage(get_assets_root_path() + args.env, "/Environment")
simulation_app.update()
stage = omni.usd.get_context().get_stage()
bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
lo, hi = bounds.GetMin(), bounds.GetMax()

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
app_utils.play(commit=True)
for _ in range(10):
    simulation_app.update()

origin = (args.origin[0], args.origin[1], 0.0)
generator = _omap.Generator(omni.physx.get_physx_interface(), omni.usd.get_context().get_stage_id())
# 引数: セルサイズ, 占有セルの値, 空きセルの値, 未知セルの値（高さの帯は set_transform の z で指定する）
generator.update_settings(args.resolution, 1.0, 0.0, 0.5)
# 引数: 原点（空き地点）, 原点からの下限オフセット, 上限オフセット（z は障害物とみなす高さの帯）
generator.set_transform(
    origin,
    (lo[0] - origin[0], lo[1] - origin[1], args.z_min),
    (hi[0] - origin[0], hi[1] - origin[1], args.z_max),
)
generator.generate2d()
dims = generator.get_dimensions()
width, height = int(dims.x), int(dims.y)
cells = np.array(generator.get_buffer(), dtype=np.float32).reshape(height, width)
# 画像左下の座標。拡張の UI と同じく、境界に半セルを足す
min_bound = [generator.get_min_bound().x + args.resolution / 2, generator.get_min_bound().y + args.resolution / 2]
max_bound = [generator.get_max_bound().x, generator.get_max_bound().y]

# omap のバッファは 1.0=占有, 0.0=空き, それ以外=未知。map_server の trinary 形式（黒=占有, 白=空き, 灰=未知）へ
image = np.full((height, width), 205, dtype=np.uint8)
image[cells == 0.0] = 254
image[cells == 1.0] = 0
# バッファは 0 行目が y 最小、0 列目が x 最大の並び（isaacsim.asset.gen.omap.utils.compute_coordinates）。
# ROS の地図画像は 0 行目が y 最大、0 列目が x 最小なので 180° 回転する（拡張の UI の既定と同じ）
image = np.rot90(image, 2)

os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
with open(args.output + ".pgm", "wb") as f:
    f.write(f"P5\n{width} {height}\n255\n".encode())
    f.write(image.tobytes())
with open(args.output + ".yaml", "w") as f:
    f.write(
        f"image: {os.path.basename(args.output)}.pgm\n"
        f"mode: trinary\n"
        f"resolution: {args.resolution}\n"
        f"origin: [{min_bound[0]:.3f}, {min_bound[1]:.3f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
print(
    f"MAP_DONE {width}x{height} origin=({min_bound[0]:.2f},{min_bound[1]:.2f}) max=({max_bound[0]:.2f},{max_bound[1]:.2f}) "
    f"occupied={int((cells == 1.0).sum())} free={int((cells == 0.0).sum())}",
    flush=True,
)

app_utils.stop()
simulation_app.close()
