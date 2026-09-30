"""状態・異常の表示文言。Nav2 などの内部用語は出さず、現場向けに言い換える（doc/ui_screens.md 方針 5）。

文言は試作画面で確認しながら調整する。
"""

from locomotion.v1 import locomotion_pb2 as pb

# 状態: (表示, 色)
STATE = {
    pb.ROBOT_STATE_NOT_READY: ("準備未完了", "#757575"),
    pb.ROBOT_STATE_IDLE: ("待機", "#2e7d32"),
    pb.ROBOT_STATE_NAVIGATING: ("移動中", "#1565c0"),
    pb.ROBOT_STATE_PAUSED: ("一時停止", "#f9a825"),
    pb.ROBOT_STATE_FAULT: ("異常停止", "#c62828"),
    pb.ROBOT_STATE_SAFETY_STOP: ("安全停止", "#c62828"),
    pb.ROBOT_STATE_MANUAL: ("手動操作", "#6a1b9a"),
    pb.ROBOT_STATE_MAPPING: ("地図作成中", "#6a1b9a"),
}

MODE = {
    pb.MODE_UPPER: "上位指示モード",
    pb.MODE_OPERATOR: "UI指示モード",
    pb.MODE_JOYSTICK: "ジョイスティックモード",
}

NAV_ACTIVITY = {
    pb.NAV_ACTIVITY_NORMAL: "通常",
    pb.NAV_ACTIVITY_RECOVERING: "回避動作中",
}
RECOVERING_NOTE = "行き詰まったため、向きを変えて経路を探しています"

SAFETY_KIND = {
    pb.SAFETY_STOP_KIND_PROTECTIVE: "保護停止",
    pb.SAFETY_STOP_KIND_EMERGENCY: "非常停止",
}

TASK_STATUS = {
    pb.TASK_STATUS_RUNNING: "実行中",
    pb.TASK_STATUS_PAUSED: "一時停止",
    pb.TASK_STATUS_FAULTED: "異常停止中",
    pb.TASK_STATUS_SUCCEEDED: "成功",
    pb.TASK_STATUS_CANCELED: "中断",
    pb.TASK_STATUS_FAILED: "失敗",
}

TASK_SOURCE = {pb.TASK_SOURCE_UPPER: "上位", pb.TASK_SOURCE_OPERATOR: "UI"}

# 対応パネル（S2）: (見出し, 何が起きたか, 現場で何をするか)
NOT_READY = (
    "準備未完了",
    "ロボットの位置が決まっていません。",
    "地図上でロボットがいる場所をタップし（ドラッグで向き）、［この位置に設定］を押してください。"
    "登録地点にいる場合は、地点を選んで設定することもできます。",
)

FAULT = {
    pb.FAULT_CATEGORY_PATH_BLOCKED: (
        "経路閉塞",
        "障害物で通れず、回避もできませんでした。",
        "ロボットの進む先の障害物をどけてから［再試行］を押してください。",
    ),
    pb.FAULT_CATEGORY_GOAL_UNREACHABLE: (
        "目標到達不能",
        "行き先が障害物の上か、立入禁止エリアの中にあります。",
        "地点の設定の見直しが必要です（導入担当者に連絡）。［タスクを中止］してください。",
    ),
    pb.FAULT_CATEGORY_LOCALIZATION_LOST: (
        "自己位置喪失",
        "ロボットが自分の位置を見失いました。",
        "地図上でロボットがいる場所を指定し直すか、手動操作で分かる場所まで動かしてください。",
    ),
    pb.FAULT_CATEGORY_DEVICE_ERROR: (
        "機器異常",
        "センサや台車の機器が応答していません。",
        "現場では直せません。サポートに連絡し、［タスクを中止］してください。",
    ),
}

SAFETY = {
    pb.SAFETY_STOP_KIND_PROTECTIVE: (
        "保護停止",
        "安全装置が働いて止まりました。",
        "周囲の安全を確認してください。安全装置が解除されると「一時停止」になり、［再開］できます。",
    ),
    pb.SAFETY_STOP_KIND_EMERGENCY: (
        "非常停止",
        "非常停止ボタンが押されました。",
        "安全を確認して非常停止ボタンを解除し、［リセット］を押してください。",
    ),
}

JOYSTICK_NOTE = "ゲームパッドでも操作できます（LB を押しながらスティック）"
UPPER_MODE_NOTE = "上位システムの指示で動作中です。一時停止・停止はヘッダのボタンで行えます。"
