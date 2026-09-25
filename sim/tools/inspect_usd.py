"""USD の中身(センサ・関節・OmniGraph・ロボットスキーマ)を一覧表示する調査用スクリプト。

使い方: ~/isaacsim/python.sh inspect_usd.py <USDのパス(assets root からの相対 or 絶対URL)> [...]
"""
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

root = get_assets_root_path()
print("ASSETS_ROOT", root)

for arg in sys.argv[1:]:
    url = arg if "://" in arg or arg.startswith("/home") else root + arg
    stage = Usd.Stage.Open(url)
    print("=" * 80)
    print("FILE", url, "OK" if stage else "FAILED")
    if not stage:
        continue
    print("defaultPrim", stage.GetDefaultPrim().GetPath(), "metersPerUnit", UsdGeom.GetStageMetersPerUnit(stage),
          "upAxis", UsdGeom.GetStageUpAxis(stage))
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetDefaultPrim())
    r = bbox.ComputeAlignedRange()
    print("BBOX min", r.GetMin(), "max", r.GetMax(), "size", r.GetSize())
    for p in stage.Traverse():
        t = p.GetTypeName()
        schemas = p.GetAppliedSchemas()
        interesting = (
            p.HasAPI(UsdPhysics.ArticulationRootAPI)
            or "Lidar" in t or "Camera" == t or "Imu" in t or "Sensor" in t
            or t.startswith("OmniGraph")
            or any("Isaac" in s or "Robot" in s or "Wheel" in s or "Holonomic" in s for s in schemas)
            or (t.endswith("Joint") and "roller" not in p.GetName().lower() and "RevoluteJoint" not in p.GetName())
        )
        if interesting:
            extra = ""
            if t == "OmniGraphNode":
                a = p.GetAttribute("node:type")
                extra = f" node:type={a.Get() if a else '?'}"
            print(f"  {t:28s} {p.GetPath()}{extra}  {[s for s in schemas if 'Material' not in s][:4]}")
app.close()
