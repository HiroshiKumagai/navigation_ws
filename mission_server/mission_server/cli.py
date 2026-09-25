"""動作確認用のコマンドラインクライアント（gRPC のみを使う。UI と同じ立場の利用者）。

使い方: .venv/bin/python -m mission_server.cli [--server localhost:50051] [--role operator] <コマンド> ...

    status                      現在の状態
    watch                       状態の変化を表示し続ける（Ctrl+C で終了）
    pose                        位置・速度を表示し続ける
    points [--map ID]           地点の一覧
    routes [--map ID]           ルートの一覧
    tasks                       実行記録の一覧
    task TASK_ID                実行記録の詳細（状態の履歴つき）
    init-pose POINT_ID | --pose X Y YAW_DEG      初期位置の設定
    mode upper|operator|joystick                  モード切替
    go POINT_ID [POINT_ID ...] [--request-id ID]   移動指示（複数なら巡回）
    go --route ROUTE_ID                           ルートで移動指示
    pause | resume | stop | retry [TASK_ID]       保持中のタスクへの操作（省略時は保持中のタスク）
    reset-safety                非常停止のリセット
    add-point POINT_ID [--pose X Y YAW_DEG] [--name NAME]   地点の登録（--pose 省略時は現在位置。導入担当者）
"""

import argparse
import math
import sys
import uuid

import grpc
from grpc_status import rpc_status
from locomotion.v1 import locomotion_pb2 as pb
from locomotion.v1 import locomotion_pb2_grpc as pb_grpc


def enum_name(enum, value, prefix):
    return enum.Name(value).removeprefix(prefix)


def format_status(s):
    parts = [f"#{s.sequence}", enum_name(pb.RobotState, s.state, "ROBOT_STATE_"), f"mode={enum_name(pb.Mode, s.mode, 'MODE_')}"]
    if s.state == pb.ROBOT_STATE_NAVIGATING:
        parts.append(enum_name(pb.NavActivity, s.nav_activity, "NAV_ACTIVITY_"))
    if s.state == pb.ROBOT_STATE_SAFETY_STOP:
        parts.append(enum_name(pb.SafetyStopKind, s.safety_stop_kind, "SAFETY_STOP_KIND_"))
    if s.state == pb.ROBOT_STATE_FAULT:
        parts.append(f"fault={enum_name(pb.FaultCategory, s.fault.category, 'FAULT_CATEGORY_')}({s.fault.detail_code})")
    loc = s.localization
    parts.append(f"map={s.active_map_id} localized={loc.localized}(xy±{loc.covariance_xy:.2f} yaw±{math.degrees(loc.covariance_yaw):.0f}°)")
    if s.HasField("active_task"):
        t = s.active_task
        parts.append(
            f"task={t.task_id[:8]} {enum_name(pb.TaskStatus, t.status, 'TASK_STATUS_')} "
            f"{t.current_index}/{len(t.point_ids)} {list(t.point_ids)}"
        )
    return "  ".join(parts)


def format_task(t, history=False):
    route = f"route={t.route_id} " if t.route_id else ""
    line = (
        f"{t.task_id}  {enum_name(pb.TaskStatus, t.status, 'TASK_STATUS_'):9s}  "
        f"{enum_name(pb.TaskSource, t.source, 'TASK_SOURCE_'):8s}  {route}{list(t.point_ids)}  {t.current_index}/{len(t.point_ids)}"
    )
    if history:
        for e in t.history:
            fault = (
                f"  {enum_name(pb.FaultCategory, e.fault.category, 'FAULT_CATEGORY_')}({e.fault.detail_code})"
                if e.HasField("fault")
                else ""
            )
            line += (
                f"\n    {e.at.ToDatetime():%H:%M:%S}  {enum_name(pb.TaskStatus, e.status, 'TASK_STATUS_'):9s}"
                f"  ({enum_name(pb.RobotState, e.robot_state, 'ROBOT_STATE_')}){fault}"
            )
    return line


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--role", default="operator", choices=["upper-system", "operator", "integrator"])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("watch")
    sub.add_parser("pose")
    p = sub.add_parser("points")
    p.add_argument("--map", default="")
    p = sub.add_parser("routes")
    p.add_argument("--map", default="")
    sub.add_parser("tasks")
    p = sub.add_parser("task")
    p.add_argument("task_id")
    p = sub.add_parser("init-pose")
    p.add_argument("point_id", nargs="?")
    p.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"))
    p = sub.add_parser("mode")
    p.add_argument("mode", choices=["upper", "operator", "joystick"])
    p = sub.add_parser("go")
    p.add_argument("point_ids", nargs="*")
    p.add_argument("--route", default="")
    p.add_argument("--request-id", default="")
    for name in ("pause", "resume", "stop", "retry"):
        p = sub.add_parser(name)
        p.add_argument("task_id", nargs="?")
    sub.add_parser("reset-safety")
    p = sub.add_parser("add-point")
    p.add_argument("point_id")
    p.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"))
    p.add_argument("--name", default="")
    args = parser.parse_args()

    channel = grpc.insecure_channel(args.server)
    md = [("x-locomotion-role", args.role)]
    robot = pb_grpc.RobotServiceStub(channel)
    tasks = pb_grpc.TaskServiceStub(channel)
    points = pb_grpc.PointServiceStub(channel)

    def active_task_id(given):
        if given:
            return given
        status = robot.GetStatus(pb.GetStatusRequest(), metadata=md)
        if not status.HasField("active_task"):
            sys.exit("保持中のタスクがありません")
        return status.active_task.task_id

    try:
        c = args.command
        if c == "status":
            print(format_status(robot.GetStatus(pb.GetStatusRequest(), metadata=md)))
        elif c == "watch":
            for s in robot.WatchStatus(pb.WatchStatusRequest(), metadata=md):
                print(format_status(s), flush=True)
        elif c == "pose":
            for u in robot.WatchPose(pb.WatchPoseRequest(rate_hz=2), metadata=md):
                print(
                    f"({u.pose.x:7.2f}, {u.pose.y:7.2f}, {math.degrees(u.pose.yaw):6.1f}°)  "
                    f"v=({u.linear_velocity_x:+.2f}, {u.linear_velocity_y:+.2f}) m/s  w={u.angular_velocity:+.2f} rad/s",
                    flush=True,
                )
        elif c == "points":
            map_id = args.map or robot.GetStatus(pb.GetStatusRequest(), metadata=md).active_map_id
            for pt in points.ListPoints(pb.ListPointsRequest(map_id=map_id), metadata=md).points:
                print(f"{pt.point_id:10s} ({pt.pose.x:7.2f}, {pt.pose.y:7.2f}, {math.degrees(pt.pose.yaw):6.1f}°)  {pt.name}")
        elif c == "routes":
            map_id = args.map or robot.GetStatus(pb.GetStatusRequest(), metadata=md).active_map_id
            for r in points.ListRoutes(pb.ListRoutesRequest(map_id=map_id), metadata=md).routes:
                print(f"{r.route_id:10s} {' -> '.join(r.point_ids):30s} {r.name}")
        elif c == "tasks":
            for t in tasks.ListTasks(pb.ListTasksRequest(), metadata=md).tasks:
                print(format_task(t))
        elif c == "task":
            print(format_task(tasks.GetTask(pb.GetTaskRequest(task_id=args.task_id), metadata=md), history=True))
        elif c == "init-pose":
            if args.pose:
                x, y, yaw = args.pose
                request = pb.SetInitialPoseRequest(pose=pb.Pose2D(x=x, y=y, yaw=math.radians(yaw)))
            elif args.point_id:
                request = pb.SetInitialPoseRequest(point_id=args.point_id)
            else:
                sys.exit("POINT_ID か --pose を指定してください")
            robot.SetInitialPose(request, metadata=md)
            print(format_status(robot.GetStatus(pb.GetStatusRequest(), metadata=md)))
        elif c == "mode":
            response = robot.SetMode(pb.SetModeRequest(mode=getattr(pb, f"MODE_{args.mode.upper()}")), metadata=md)
            print("->", enum_name(pb.RobotState, response.resulting_state, "ROBOT_STATE_"))
        elif c == "go":
            request = pb.StartTaskRequest(
                request_id=args.request_id or uuid.uuid4().hex, point_ids=args.point_ids, route_id=args.route
            )
            print(format_task(tasks.StartTask(request, metadata=md).task))
        elif c in ("pause", "resume", "stop", "retry"):
            method = {"pause": tasks.PauseTask, "resume": tasks.ResumeTask, "stop": tasks.StopTask, "retry": tasks.RetryTask}[c]
            print(format_task(method(pb.TaskCommandRequest(task_id=active_task_id(args.task_id)), metadata=md).task))
        elif c == "reset-safety":
            robot.ResetSafetyStop(pb.ResetSafetyStopRequest(), metadata=md)
            print("リセットを要求しました")
        elif c == "add-point":
            map_id = robot.GetStatus(pb.GetStatusRequest(), metadata=md).active_map_id
            request = pb.CreatePointRequest(map_id=map_id, point_id=args.point_id, name=args.name)
            if args.pose:
                x, y, yaw = args.pose
                request.pose.CopyFrom(pb.Pose2D(x=x, y=y, yaw=math.radians(yaw)))
            else:
                request.use_current_pose = True
            pt = points.CreatePoint(request, metadata=md)
            print(f"登録しました: {pt.point_id} ({pt.pose.x:.2f}, {pt.pose.y:.2f}, {math.degrees(pt.pose.yaw):.1f}°)")
    except grpc.RpcError as e:
        message = f"拒否されました: {e.code().name}: {e.details()}"
        status = rpc_status.from_call(e)
        if status and status.details:
            detail = pb.RejectionDetail()
            if status.details[0].Unpack(detail):
                message += (
                    f"（状態 {enum_name(pb.RobotState, detail.current_state, 'ROBOT_STATE_')}, "
                    f"モード {enum_name(pb.Mode, detail.current_mode, 'MODE_')}）"
                )
        sys.exit(message)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
