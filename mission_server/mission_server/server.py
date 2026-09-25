"""ミッションサーバの起動: 保存部・ROS 2 アダプタ・中核・gRPC サービスを組み立てる。

使い方: .venv/bin/python -m mission_server [--port 50051] [--data-dir data] [--initial-mode operator]
"""

import argparse
import signal
import threading
from concurrent import futures
from pathlib import Path

import grpc
import rclpy

from .core.model import Mode
from .core.state_machine import LocomotionCore
from .grpc_api.services import ServerContext, StatusHub, add_services
from .ros_adapter import RosAdapter
from .store import Store

DEFAULT_DATA_DIR = Path(__file__).resolve().parents[2] / "data"


def build(store, adapter, initial_mode=Mode.OPERATOR):
    """中核と gRPC で共有するものを組み立てる（テストでは adapter を差し替える）"""
    hub = StatusHub()
    core = LocomotionCore(
        initial_mode=initial_mode,
        active_map_id=store.active_map_id,
        point_exists=store.point_exists,
        map_exists=store.map_exists,
        on_effect=adapter.handle_effect,
        on_status=hub.publish,
    )
    adapter.attach(core)
    return ServerContext(core, adapter, store, hub)


def serve(ctx, port, max_workers=16):
    # ポートの共有（SO_REUSEPORT）を禁止し、同じポートで 2 つ目のサーバが起動しないようにする
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers), options=[("grpc.so_reuseport", 0)])
    add_services(server, ctx)
    bound = server.add_insecure_port(f"[::]:{port}")
    server.start()
    return server, bound


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--data-dir", default=str(DEFAULT_DATA_DIR))
    parser.add_argument("--initial-mode", choices=["upper", "operator", "joystick"], default="operator")
    parser.add_argument("--no-sim-time", action="store_true", help="実機用（use_sim_time を使わない）")
    args = parser.parse_args()

    rclpy.init()
    store = Store(args.data_dir)
    adapter = RosAdapter(
        store,
        use_sim_time=not args.no_sim_time,
        localization_level=store.settings.get("default_localization_threshold", "STANDARD"),
    )
    ctx = build(store, adapter, Mode[args.initial_mode.upper()])
    adapter.start()
    try:
        server, port = serve(ctx, args.port)
    except RuntimeError as e:
        raise SystemExit(f"ポート {args.port} を使えません（別のミッションサーバが起動中？）: {e}")
    print(f"ミッションサーバを起動しました（gRPC :{port}, 地図 {store.active_map_id}）", flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    server.stop(grace=2).wait()
    adapter.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
