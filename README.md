# navigation_ws — セミヒューマノイド ロコモーション開発環境

Nav2 を土台に、ミッションサーバと UI を載せて移動台車を運用するソフトウェアの開発ワークスペース。
仕様は [doc/locomotion_spec.md](doc/locomotion_spec.md)、API は [proto/locomotion/v1/locomotion.proto](proto/locomotion/v1/locomotion.proto) を正とする。
開発の進め方は [doc/roadmap.md](doc/roadmap.md)。

現在は、Isaac Sim 上の全方向移動台車（O3dyn）を Nav2 とゲームパッドで動かせる段階。
異常（安全停止・機器異常・自己位置喪失）を再現するテストパネルがある。
ミッションサーバは状態遷移の中核と ROS 2 との接続まで実装済み（`mission_server/`）。gRPC と UI は未着手。

## ミッションサーバ

```bash
scripts/gen_proto.sh                                   # proto を変更したら gRPC のコードを再生成（mission_server/gen/）
cd mission_server && ../.venv/bin/python -m pytest -q  # 単体テスト（仕様書 4 章の遷移表の各行ほか）
```

| パス | 内容 |
|---|---|
| `proto/locomotion/v1/locomotion.proto` | gRPC API の定義 |
| `mission_server/mission_server/core/` | 状態遷移とタスク管理（ROS 2 にも gRPC にも依存しない） |
| `mission_server/tests/test_transitions.py` | 遷移表の各行のテスト（テスト名の番号 = 表の行） |
| `mission_server/tests/test_rules.py` | 受理・拒否、権限、冪等性、異常検知のテスト |
| `mission_server/mission_server/ros_adapter.py` | ROS 2 との接続（Nav2・AMCL・安全系・指令切替ノード・map_server） |
| `mission_server/mission_server/store.py` | 地図・地点・ロボット設定の保存（`data/`） |
| `mission_server/tools/try_with_sim.py` | 中核 + ROS 2 接続をシミュレーションで試す（移動・一時停止・安全停止） |
| `data/` | 地図・地点・ロボット設定（シミュレーション用の倉庫の地図と地点 home, A〜D） |

## クイックスタート

```bash
cd ~/future_ws/navigation_ws
./start_sim.sh            # Isaac Sim（画面あり）→ Nav2 + rviz2 + ゲームパッドの順に起動。Ctrl+C で全て終了
```

- 初回はシェーダのコンパイルで Isaac Sim の起動に数分かかる
- `ROS_DOMAIN_ID` は既定で **42**。別のターミナルから ROS 2 を触るときは同じ値にする

```bash
source /opt/ros/jazzy/setup.bash && source ~/future_ws/navigation_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

オプション: `--panel`（テストパネルも起動）, `--headless`（Isaac Sim を画面なし）, `--no-nav`（Isaac Sim のみ）, `--no-rviz`, `--no-joy`, `--mode joystick`

## 操作

### Nav2 で移動する
- rviz2 の「Nav2 Goal」で地図上をクリック＆ドラッグ
- またはコマンド: `python3 sim/tools/send_goal.py X Y [YAW_DEG]`

### ゲームパッド（Logicool F310、背面スイッチは X）
| 操作 | 割り当て |
|---|---|
| 動かす（押している間だけ） | **LB を押しながら**スティック |
| 前後 / 横移動 | 左スティック 上下 / 左右 |
| 旋回 | 右スティック 左右 |
| 高速 | LB + RB |

ゲームパッドで動かすには、指令切替ノードをジョイスティックモードにする（起動時に `--mode joystick` でも可）。

```bash
ros2 service call /cmd_vel_switch/set_joystick_mode std_srvs/srv/SetBool "{data: true}"   # ゲームパッド
ros2 service call /cmd_vel_switch/set_joystick_mode std_srvs/srv/SetBool "{data: false}"  # Nav2
```

### テストパネル（異常の再現）
`./start_sim.sh --panel`、または単体で `.venv/bin/python sim/test_panel.py`。製品の UI とは別の、シミュレーション専用の画面。

| 操作 | 再現されること | ミッションサーバから見えるもの |
|---|---|---|
| 安全系: 保護停止 | 台車が速度指令を受け付けない | `/safety/state` = 1 |
| 安全系: 非常停止 | 同上。スイッチを戻しても解除されず、リセットが必要 | `/safety/state` = 2 |
| 機器異常: LiDAR 左前 / 右後 | その LiDAR の配信が止まる | スキャンの途絶 |
| 機器異常: 台車ドライバ | 速度指令を受け付けず、`/odom`・`/joint_states` が止まる | オドメトリの途絶 |
| ワープ | 地図で指定した姿勢へ瞬間移動。`/odom` は追従しない | 自己位置喪失（AMCL の推定がずれる） |

地図には真値（青）と AMCL の推定（橙）が表示され、推定誤差も数値で出る。

### その他のツール（`sim/tools/`）
| スクリプト | 用途 |
|---|---|
| `send_goal.py X Y [YAW]` | Nav2 に目標を送り、結果を表示 |
| `set_initial_pose_from_sim.py` | シミュレーションの真値（`/sim/ground_truth`）を AMCL の初期位置として送る（Nav2 だけ再起動したとき・ワープ後の復旧用） |
| `check_omni_motion.py` | 前進・横移動・旋回の指令で全方向移動を確認（Nav2 なしで使う） |
| `generate_map.py` | 環境 USD から Nav2 用の地図を生成（Isaac Sim の python.sh で実行） |
| `inspect_usd.py` | USD のセンサ・関節・OmniGraph を一覧表示（調査用） |

## 構成

```
Isaac Sim（sim/o3dyn_bringup.py）                    ROS 2（src/o3dyn_navigation）
  O3dyn 台車（メカナムの逆運動学）  ← /cmd_vel ─── collision_monitor ← /cmd_vel_switched ← cmd_vel_switch ← /cmd_vel_smoothed ← Nav2
  2D LiDAR x2 ─ /scan_front_left, /scan_rear_right → scan_merger → /scan → AMCL, コストマップ     ↑
  /odom, /tf, /tf_static, /clock, /joint_states                          ゲームパッド → joy → teleop_twist_joy → /cmd_vel_joy
```

### ROS 2 インタフェース（シミュレーションが提供するもの）
| トピック | 型 | 内容 |
|---|---|---|
| `/cmd_vel`（購読） | `geometry_msgs/Twist` | 台車への速度指令。`linear.x`, `linear.y`, `angular.z` を使う |
| `/odom` | `nav_msgs/Odometry` | `odom` → `base_link`（シミュレーションの真値。ただしワープでは動かない） |
| `/tf` | | `odom` → `base_link` |
| `/tf_static` | | `base_link` → `lidar_front_left`, `lidar_rear_right` |
| `/scan_front_left`, `/scan_rear_right` | `sensor_msgs/LaserScan` | 2D LiDAR（360°, 0.05〜40 m, 10 Hz） |
| `/joint_states` | `sensor_msgs/JointState` | 全関節 |
| `/clock` | `rosgraph_msgs/Clock` | シミュレーション時刻（ROS 側は `use_sim_time: true`） |
| `/safety/state` | `std_msgs/UInt8` | 安全系の状態 0=正常, 1=保護停止, 2=非常停止（transient local）。**実機の安全 PLC の仕様が決まるまでの仮の形式** |
| `/safety/reset`（購読） | `std_msgs/Empty` | 非常停止のリセット（ミッションサーバの ResetSafetyStop から出す想定） |

試験用（テストパネルが使う。ミッションサーバは使わない）

| トピック / サービス | 型 | 内容 |
|---|---|---|
| `/sim/safety_switch`（購読） | `std_msgs/UInt8` | 模擬安全スイッチ 0=解除, 1=保護停止, 2=非常停止 |
| `/sim/fault/{lidar_front_left, lidar_rear_right, drive}` | `std_srvs/SetBool` | 機器異常の ON/OFF |
| `/sim/warp`（購読） | `geometry_msgs/PoseStamped` | map 座標の姿勢へ瞬間移動 |
| `/sim/ground_truth` | `nav_msgs/Odometry` | 真値（map → base_link） |
| `/sim/status` | `std_msgs/String` | 試験用の状態（JSON） |

座標系: Isaac Sim のワールド座標 = `map` フレーム。ロボットはワールド原点に置かれ、`odom` の原点もそこになる。

### ディレクトリ
| パス | 内容 |
|---|---|
| `start_sim.sh` | 環境の一括起動 |
| `doc/` | 仕様書と API 定義（proto） |
| `sim/o3dyn_bringup.py` | Isaac Sim のシーン構築と ROS 2 接続（O3dyn・LiDAR・OmniGraph） |
| `sim/sim_test_hooks.py` | 異常再現の仕組み（安全系・機器異常・ワープ）。Isaac Sim の中で動く |
| `sim/test_panel.py` | テストパネル（PySide6） |
| `sim/tools/` | 開発用ツール |
| `sim/rviz/o3dyn.rviz` | Nav2 なしでセンサを見るための rviz 設定 |
| `src/o3dyn_navigation/` | ROS 2 パッケージ: Nav2 設定・地図・scan_merger・cmd_vel_switch・起動ファイル |
| `dev_setting/` | 環境構築スクリプト（下記） |
| `requirements.txt` | `.venv` に入れる Python パッケージ |
| `proto/`, `mission_server/`, `scripts/gen_proto.sh` | API 定義とミッションサーバ（上記） |
| `.claude/skills/` | Claude Code 用の Isaac Sim スキル（`~/isaacsim/skills` へのリンク） |

## 環境

| 項目 | バージョン・設定 |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy（`/opt/ros/jazzy`） |
| Isaac Sim | 6.1.0（`~/isaacsim`、zip 版。`ISAACSIM_PATH` / `ISAAC_SIM_DIR` を `~/.bashrc` に設定） |
| NVIDIA ドライバ | 595 系（Isaac Sim 6.x は 595.58.03 以上が必要） |
| GPU | RTX 3090 Ti（24 GB） |

### 構築手順（`dev_setting/`、番号順に実行）
| スクリプト | 内容 |
|---|---|
| `01_upgrade_to_noble.sh` | Ubuntu 22.04 → 24.04 |
| `01b_post_upgrade_fix.sh` | アップグレード後の外部リポジトリ・パッケージの整理 |
| `02_install_ros2_jazzy.sh` | ROS 2 Jazzy |
| `03_install_turtlebot3_sim.sh` | TurtleBot3 + Gazebo（Nav2 もこのとき入る） |
| `04_update_nvidia_driver.sh` | NVIDIA ドライバ 595（要再起動） |
| `05_install_isaac_sim.sh` | Isaac Sim 6.1.0 |
| `06_setup_python_env.sh` | Python 環境（`.venv`: PySide6, grpcio ほか）、Qt の表示に必要なライブラリ、gRPC コード生成、mission_server の登録 |

ワークスペースのビルド: `colcon build --symlink-install`

## シミュレーションの前提と既知の制約

| 項目 | 内容 |
|---|---|
| 台車 | Isaac Sim 付属の O3dyn（Fraunhofer IML。約 2.07 × 1.48 m、全方向移動）。実機はメカナム台車 |
| ホイール制御 | 付属の HolonomicController は速度が約 1/3・旋回が逆符号になるため使わず、メカナムの逆運動学を自前で計算 |
| LiDAR | 左前・右後ろの角に 1 台ずつ。SICK nanoScan3 モデルは描画フレームにスキャンが収まらず約 3 割欠けるため、回転型 2D LiDAR（Example_Rotary_2D）で代用 |
| オドメトリ | シミュレーションの真値（ワープ分は除く）。実機のホイールオドメトリの誤差は再現していない |
| 速度指令 | シミュレーションは `/cmd_vel` の最後の値を保持する。停止は cmd_vel_switch のタイムアウト（0.5 秒）に頼っている |
| 地図 | `Simple_Warehouse/warehouse.usd` の当たり判定から生成（`maps/warehouse.*`）。広く特徴が少ないため、左側で AMCL が x 方向に最大 0.4 m ほどずれる |
| 速度上限 | 台車 2.0 m/s（仮）, Nav2 0.5 m/s, ゲームパッド 0.3（高速 0.5）m/s |

## トラブルシューティング

- **トピックが見えない**: `ROS_DOMAIN_ID` が 42 になっているか。Isaac Sim は Play 中のみ配信する
- **Nav2 の起動が「transform from base_link to map」で止まる**: ロボットが原点以外にいる状態で Nav2 だけを起動し直したとき。`sim/tools/set_initial_pose_from_sim.py` を実行してから
  `ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"` → `"{command: 0}"`
- **Isaac Sim のログ**: `log/sim/isaacsim_*.log`、テストパネルのログ: `log/sim/test_panel.log`
- **テストパネルが「Could not load the Qt platform plugin "xcb"」で起動しない**: `dev_setting/06_setup_python_env.sh` を実行（`libxcb-cursor0` が必要）
