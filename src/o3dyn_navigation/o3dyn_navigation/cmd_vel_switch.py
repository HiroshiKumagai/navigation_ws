"""指令切替ノード（仕様書 2 章）。Nav2 とジョイスティックの速度指令のうち、モードに応じて一方だけを通す。

- 入力: nav_cmd_vel（Nav2 の velocity_smoother 出力）, joy_cmd_vel（teleop_twist_joy 出力）
- 出力: cmd_vel_out（collision_monitor の入力。手動操作にも減速・停止を効かせる）
- モード切替: サービス ~/set_joystick_mode（std_srvs/SetBool。true=ジョイスティック, false=ナビゲーション）
  本来はミッションサーバが停止中にだけ切り替える。この段階では開発用に直接呼べるようにしている
- 選択中の入力が timeout 秒途絶えたら、停止指令を 1 回出して以後は何も出さない
- 現在のモードを ~/mode（std_msgs/String, transient local）で配信する
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

NAVIGATION = "navigation"
JOYSTICK = "joystick"


class CmdVelSwitch(Node):
    def __init__(self):
        super().__init__("cmd_vel_switch")
        self.mode = JOYSTICK if self.declare_parameter("initial_mode", NAVIGATION).value == JOYSTICK else NAVIGATION
        self.timeout = self.declare_parameter("timeout", 0.5).value

        self.publisher = self.create_publisher(Twist, "cmd_vel_out", 10)
        self.mode_publisher = self.create_publisher(
            String, "~/mode", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.create_subscription(Twist, "nav_cmd_vel", lambda m: self._on_cmd(NAVIGATION, m), 10)
        self.create_subscription(Twist, "joy_cmd_vel", lambda m: self._on_cmd(JOYSTICK, m), 10)
        self.create_service(SetBool, "~/set_joystick_mode", self._on_set_mode)
        self.create_timer(0.05, self._check_timeout)

        self.last_cmd_time = None
        self._publish_mode()

    def _on_cmd(self, source, msg):
        if source != self.mode:
            return
        self.last_cmd_time = self.get_clock().now()
        self.publisher.publish(msg)

    def _check_timeout(self):
        if self.last_cmd_time is None:
            return
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > self.timeout * 1e9:
            self._stop()

    def _stop(self):
        self.publisher.publish(Twist())
        self.last_cmd_time = None

    def _on_set_mode(self, request, response):
        new_mode = JOYSTICK if request.data else NAVIGATION
        if new_mode != self.mode:
            self._stop()
            self.mode = new_mode
            self._publish_mode()
            self.get_logger().info(f"モードを {self.mode} に切り替えました")
        response.success = True
        response.message = self.mode
        return response

    def _publish_mode(self):
        self.mode_publisher.publish(String(data=self.mode))


def main():
    rclpy.init()
    node = CmdVelSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
