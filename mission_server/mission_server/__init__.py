"""ロコモーション ミッションサーバ（仕様: doc/locomotion_spec.md）。

    core/          状態遷移とタスク管理（ROS 2 にも gRPC にも依存しない）
    ros_adapter    中核の副作用を ROS 2 で実行し、ROS 2 の観測を中核に伝える
    store          地図・地点・ロボット設定の保存
    grpc_api/      gRPC サービス（locomotion.v1）
    server         組み立てと起動（python -m mission_server）
"""
