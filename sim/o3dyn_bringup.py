"""O3dyn（全方向移動台車）を倉庫シーンに置き、ROS 2 から操作できるようにする起動スクリプト。

使い方:
    ~/isaacsim/python.sh sim/o3dyn_bringup.py [--headless] [--test-frames N]

ROS 2 インタフェース（Play 中のみ有効）:
    購読   /cmd_vel               geometry_msgs/Twist   linear.x, linear.y, angular.z を使用
    配信   /odom                  nav_msgs/Odometry     odom -> base_link（シミュレーション上の真値）
           /tf                    odom -> base_link
           /tf_static             base_link -> lidar_front_left, lidar_rear_right
           /scan_front_left       sensor_msgs/LaserScan 2D LiDAR（360°, 0.05〜40 m, 10 Hz）
           /scan_rear_right       sensor_msgs/LaserScan 2D LiDAR（360°, 0.05〜40 m, 10 Hz）
           /joint_states          sensor_msgs/JointState
           /clock                 rosgraph_msgs/Clock
    試験用（異常の再現）は sim_test_hooks.py を参照（/safety/state, /sim/* ほか）

ホイール速度は、付属の HolonomicController ではなくメカナムの逆運動学で計算する。
HolonomicController は力のつり合いを解く方式のため、O3dyn では速度が約 1/3 になり旋回の符号も逆になる。
"""

import argparse
import math

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="画面なしで起動する")
parser.add_argument("--test-frames", type=int, default=0, help="指定フレーム数だけ実行して終了する（0 なら終了しない）")
parser.add_argument(
    "--env",
    default="/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    help="環境 USD（アセットルートからの相対パス）",
)
parser.add_argument(
    "--spawn",
    type=float,
    nargs=2,
    default=[0.0, 0.0],
    metavar=("X", "Y"),
    help="ロボットの初期位置 [m]（ワールド座標 = map フレーム）",
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

ROBOT_PATH = "/O3dyn"
BASE_LINK_PATH = f"{ROBOT_PATH}/base_link"
GRAPH_PATH = "/ROS2Graph"

# 台車の速度上限（仮値。実機の仕様が決まったら合わせる）
MAX_LINEAR_SPEED = 2.0  # [m/s]
MAX_ANGULAR_SPEED = 1.5  # [rad/s]

# LiDAR の取り付け（base_link 基準）。車体外形 x: -1.07〜0.96, y: ±0.72 の対角の角に、外向きに置く
LIDARS = [
    # (名前, 位置 [m], 向き yaw [deg])
    ("front_left", (0.99, 0.75, 0.25), 45.0),
    ("rear_right", (-1.10, -0.75, 0.25), -135.0),
]

assets_root = get_assets_root_path()
stage_utils.add_reference_to_stage(assets_root + args.env, "/Environment")
robot_prim = stage_utils.add_reference_to_stage(assets_root + "/Isaac/Robots/Fraunhofer/O3dyn/o3dyn.usd", ROBOT_PATH)
# O3dyn は translate の xformOp を持っているため、XformCommonAPI ではなく既存の op を書き換える
robot_xform = UsdGeom.Xformable(robot_prim)
translate_op = next(
    (op for op in robot_xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None
) or robot_xform.AddTranslateOp()
translate_op.Set(type(translate_op.Get() or Gf.Vec3d())(args.spawn[0], args.spawn[1], 0.0))
simulation_app.update()

# ロボットは無回転で置く前提（LiDAR の向きをワールド座標で与えるため）
lidar_prims = {}
for name, (x, y, z), yaw_deg in LIDARS:
    half = math.radians(yaw_deg) / 2.0
    lidar = Lidar.create(
        path=f"{BASE_LINK_PATH}/lidar_{name}",
        # SICK_nanoScan3 はソリッドステート型で、1 スキャン（約 30 ms）が描画フレームに収まらず
        # LaserScan の約 3 割が欠けるため、回転型の 2D LiDAR を安全スキャナ相当の距離設定で使う
        config="Example_Rotary_2D",
        attributes={"omni:sensor:Core:nearRangeM": 0.05, "omni:sensor:Core:farRangeM": 40.0},
        translations=[[x, y, z]],
        orientations=[[math.cos(half), 0.0, 0.0, math.sin(half)]],
    )
    lidar_prims[name] = lidar.paths[0]

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()

# base_link から見た各 LiDAR の姿勢（静的 TF 用）
stage = stage_utils.get_current_stage()
xform_cache = UsdGeom.XformCache()
base_inv = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(BASE_LINK_PATH)).GetInverse()
lidar_tf = {}
for name, path in lidar_prims.items():
    m = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)) * base_inv
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    lidar_tf[name] = ([t[0], t[1], t[2]], [q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()])

keys = og.Controller.Keys
nodes = [
    ("Tick", "omni.graph.action.OnPlaybackTick"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    # /cmd_vel -> （メインループで逆運動学）-> ホイール関節
    ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ("HoloSetup", "isaacsim.robot.wheeled_robots.HolonomicRobotUsdSetup"),
    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
    # オドメトリと odom -> base_link（台車ドライバ異常の模擬中は DriveGate で止める）
    ("DriveGate", "omni.graph.action.Branch"),
    ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
    ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    ("PubOdomTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ("PubJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
]
connect = [
    ("Tick.outputs:tick", "PubClock.inputs:execIn"),
    ("Context.outputs:context", "PubClock.inputs:context"),
    ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
    ("Tick.outputs:tick", "SubTwist.inputs:execIn"),
    ("Context.outputs:context", "SubTwist.inputs:context"),
    ("Tick.outputs:tick", "ArticulationController.inputs:execIn"),
    ("HoloSetup.outputs:wheelDofNames", "ArticulationController.inputs:jointNames"),
    ("Tick.outputs:tick", "DriveGate.inputs:execIn"),
    ("DriveGate.outputs:execTrue", "ComputeOdom.inputs:execIn"),
    ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
    ("ComputeOdom.outputs:execOut", "PubOdomTf.inputs:execIn"),
    ("Context.outputs:context", "PubOdom.inputs:context"),
    ("Context.outputs:context", "PubOdomTf.inputs:context"),
    ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
    ("SimTime.outputs:simulationTime", "PubOdomTf.inputs:timeStamp"),
    ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
    ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
    # 姿勢（position / orientation）は sim_test_hooks がワープ補正を掛けて毎フレーム書き込む
    ("DriveGate.outputs:execTrue", "PubJointState.inputs:execIn"),
    ("Context.outputs:context", "PubJointState.inputs:context"),
    ("SimTime.outputs:simulationTime", "PubJointState.inputs:timeStamp"),
]
values = [
    ("SubTwist.inputs:topicName", "cmd_vel"),
    ("DriveGate.inputs:condition", True),
    ("HoloSetup.inputs:usePath", True),
    ("HoloSetup.inputs:robotPrimPath", ROBOT_PATH),
    ("HoloSetup.inputs:comPrimPath", f"{BASE_LINK_PATH}/com_offset"),
    ("ArticulationController.inputs:robotPath", ROBOT_PATH),
    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
    ("PubOdom.inputs:topicName", "odom"),
    ("PubOdom.inputs:odomFrameId", "odom"),
    ("PubOdom.inputs:chassisFrameId", "base_link"),
    ("PubOdomTf.inputs:parentFrameId", "odom"),
    ("PubOdomTf.inputs:childFrameId", "base_link"),
    ("PubJointState.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
    ("PubJointState.inputs:topicName", "joint_states"),
]

# LiDAR ごとに LaserScan 配信と base_link -> lidar の静的 TF
for name, path in lidar_prims.items():
    rp, scan, tf = f"RenderProduct_{name}", f"PubScan_{name}", f"PubLidarTf_{name}"
    nodes += [
        (rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        (scan, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        (tf, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ]
    connect += [
        ("Tick.outputs:tick", f"{rp}.inputs:execIn"),
        (f"{rp}.outputs:execOut", f"{scan}.inputs:execIn"),
        (f"{rp}.outputs:renderProductPath", f"{scan}.inputs:renderProductPath"),
        ("Context.outputs:context", f"{scan}.inputs:context"),
        ("Tick.outputs:tick", f"{tf}.inputs:execIn"),
        ("Context.outputs:context", f"{tf}.inputs:context"),
        ("SimTime.outputs:simulationTime", f"{tf}.inputs:timeStamp"),
    ]
    translation, rotation = lidar_tf[name]
    values += [
        (f"{rp}.inputs:cameraPrim", [usdrt.Sdf.Path(path)]),
        (f"{scan}.inputs:type", "laser_scan"),
        (f"{scan}.inputs:topicName", f"scan_{name}"),
        (f"{scan}.inputs:frameId", f"lidar_{name}"),
        (f"{tf}.inputs:parentFrameId", "base_link"),
        (f"{tf}.inputs:childFrameId", f"lidar_{name}"),
        (f"{tf}.inputs:translation", translation),
        (f"{tf}.inputs:rotation", rotation),
        (f"{tf}.inputs:staticPublisher", True),
        (f"{tf}.inputs:topicName", "tf_static"),
    ]

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values},
)
simulation_app.update()


class MecanumKinematics:
    """車体速度 (vx, vy, wz) から各ホイールの回転速度 [rad/s] を求める逆運動学。

    ホイール i の位置 (x_i, y_i)、半径 r、ローラーの向き a_i（±1）を用いて
        omega_i = (vx + a_i * vy + (a_i * x_i - y_i) * wz) / r
    ホイール配置・半径・ローラー角は HolonomicRobotUsdSetup が USD から読み取った値を使う。
    """

    def __init__(self, graph_path):
        def read(name):
            return og.Controller.attribute(f"{graph_path}/HoloSetup.outputs:{name}").get()

        self.radius = [float(r) for r in read("wheelRadius")]
        self.positions = [(float(p[0]), float(p[1])) for p in read("wheelPositions")]
        # ローラー角 -135°/45° は横方向係数 -1、-45°/135° は +1
        self.roller_sign = [-round(math.sin(math.radians(2.0 * a))) for a in read("mecanumAngles")]
        self.twist_linear = og.Controller.attribute(f"{graph_path}/SubTwist.outputs:linearVelocity")
        self.twist_angular = og.Controller.attribute(f"{graph_path}/SubTwist.outputs:angularVelocity")
        self.velocity_command = og.Controller.attribute(f"{graph_path}/ArticulationController.inputs:velocityCommand")

    def update(self, enabled=True):
        if not enabled:
            self.velocity_command.set([0.0] * len(self.radius))
            return
        vx, vy, _ = self.twist_linear.get()
        wz = self.twist_angular.get()[2]
        speed = math.hypot(vx, vy)
        if speed > MAX_LINEAR_SPEED:
            vx, vy = vx * MAX_LINEAR_SPEED / speed, vy * MAX_LINEAR_SPEED / speed
        wz = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, wz))
        self.velocity_command.set(
            [
                (vx + a * vy + (a * x - y) * wz) / r
                for (x, y), r, a in zip(self.positions, self.radius, self.roller_sign)
            ]
        )


app_utils.play()
simulation_app.update()
kinematics = MecanumKinematics(GRAPH_PATH)
from sim_test_hooks import SimTestHooks  # noqa: E402

test_hooks = SimTestHooks(GRAPH_PATH, ROBOT_PATH, [name for name, _, _ in LIDARS])
spawned = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(BASE_LINK_PATH)).ExtractTranslation()
print(f"O3DYN_BRINGUP_READY base_link world=({spawned[0]:.2f}, {spawned[1]:.2f})", flush=True)

frame = 0
while simulation_app.is_running():
    test_hooks.before_update()
    kinematics.update(enabled=test_hooks.drive_enabled)
    simulation_app.update()
    test_hooks.after_update()
    frame += 1
    if args.test_frames and frame >= args.test_frames:
        break

test_hooks.shutdown()
app_utils.stop()
simulation_app.close()
