#!/usr/bin/env python3.12
"""摇杆 -> 机器人行走遥控（PC 端，纯 std_msgs，不产生机器人消息类型依赖）。

订阅 a3_xrt_bridge 发布的摇杆（Float32MultiArray data=[x, y]）：
  /pico_left_controller/axis   x=左右(+右) y=前后(+前)
  /pico_right_controller/axis  仅用 x（左右拨 -> 原地旋转）

映射（官方定义 +forward=前进 +lateral=左移 +angular=左转；PICO/Unity 摇杆
x 轴 +为右，故 lat/ang 默认取负号，方向反了改参数即可）：
  forward_velocity = left.y   * fwd_sign * max_scale
  lateral_velocity = left.x   * lat_sign * max_scale
  angular_velocity = right.x  * ang_sign * max_scale

输出 /teleop/locomotion_cmd（std_msgs/Float32MultiArray [fwd, lat, ang]），
由机器人端 locomotion_relay.py 转成官方 RosMsgWrapper 发给 MC
（话题 /motion/control/locomotion_velocity/pb_3A...Channel，仅 MOTION 状态生效）。

安全设计（默认不动）：
  - enable 参数默认 false：不发任何指令；显式 -p enable:=true 才输出
  - 死区 deadzone（默认 0.15）：摇杆回中即视为零
  - 松开摇杆自动补 zero_frames 帧（默认 3）全零再静默
  - 摇杆数据超时 stale_timeout（默认 0.3s）强制按零处理（链路断开自动停车）
  - max_scale 默认 0.3 保守起步，范围 0~1

与手臂流式遥操作互不锁定，可同时使用（全身遥操作）。
机器人真正走起来前请确保：MC MOTION 状态、周围安全、并已征得操作者同意。

用法（与遥操作同域）：
  source setup_teleop_env.bash && export ROS_DOMAIN_ID=42
  /usr/bin/python3.12 scripts/locomotion_teleop.py                      # 默认禁用（只监视）
  /usr/bin/python3.12 scripts/locomotion_teleop.py --ros-args -p enable:=true
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class LocomotionTeleop(Node):
    def __init__(self):
        super().__init__("locomotion_teleop")
        self.declare_parameter("enable", False)
        self.declare_parameter("max_scale", 0.3)
        self.declare_parameter("deadzone", 0.15)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("zero_frames", 3)
        self.declare_parameter("warmup_s", 8.0)
        self.declare_parameter("stale_timeout", 0.3)
        self.declare_parameter("fwd_sign", 1.0)
        self.declare_parameter("lat_sign", -1.0)
        self.declare_parameter("ang_sign", -1.0)
        self._enable = bool(self.get_parameter("enable").value)
        self._scale = float(self.get_parameter("max_scale").value)
        self._deadzone = float(self.get_parameter("deadzone").value)
        self._zero_frames = int(self.get_parameter("zero_frames").value)
        self._warmup = float(self.get_parameter("warmup_s").value)
        self._warm_until = None     # 预热截止时刻（首个 tick 设定）
        self._stale = float(self.get_parameter("stale_timeout").value)
        self._signs = (
            float(self.get_parameter("fwd_sign").value),
            float(self.get_parameter("lat_sign").value),
            float(self.get_parameter("ang_sign").value),
        )

        self._axis = {"left": None, "right": None}   # 最近一帧 [x, y]
        self._last_rx = 0.0                          # 最近一次摇杆数据时刻（秒）
        self._zeros_left = 0                         # 松杆后待补的零帧数
        self._active = False                         # 上次发布的指令是否非零

        self.create_subscription(
            Float32MultiArray, "/pico_left_controller/axis",
            self._on_axis("left"), 10)
        self.create_subscription(
            Float32MultiArray, "/pico_right_controller/axis",
            self._on_axis("right"), 10)
        self._pub = self.create_publisher(
            Float32MultiArray, "/teleop/locomotion_cmd", 10)
        self.create_timer(
            1.0 / float(self.get_parameter("rate_hz").value), self._tick)

        state = "已启用" if self._enable else "已禁用（-p enable:=true 才发指令）"
        self.get_logger().info(
            f"摇杆行走遥控就绪：{state}，max_scale={self._scale} "
            f"deadzone={self._deadzone} rate={self.get_parameter('rate_hz').value}Hz"
        )
        if self._enable:
            # 预热：启动后 warmup_s 秒内持续发零（摇杆未动，机器人不会走），
            # 保证跨 WiFi 两段 DDS 发现完成、链路可被验证，避免首帧夭折。
            # _warm_until 保持 None，由首个 tick 置为 now + warmup。
            pass

    def _on_axis(self, side):
        def cb(msg: Float32MultiArray):
            if len(msg.data) < 2:
                return
            self._axis[side] = (float(msg.data[0]), float(msg.data[1]))
            self._last_rx = self.get_clock().now().nanoseconds * 1e-9
        return cb

    def _apply(self, v, sign):
        v = 0.0 if abs(v) < self._deadzone else max(-1.0, min(1.0, v))
        return v * sign * self._scale

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._warm_until is None:
            self._warm_until = now + self._warmup
        warm = now < self._warm_until   # 预热期：持续发零，等跨网段 DDS 发现完成
        stale = (now - self._last_rx) > self._stale
        if stale or self._axis["left"] is None or self._axis["right"] is None:
            lx = ly = rx = 0.0
        else:
            lx, ly = self._axis["left"]
            rx, _ry = self._axis["right"]
        cmd = (self._apply(ly, self._signs[0]),   # forward  <- 左杆 y
               self._apply(lx, self._signs[1]),   # lateral  <- 左杆 x
               self._apply(rx, self._signs[2]))   # angular  <- 右杆 x
        nonzero = any(abs(v) > 1e-6 for v in cmd)

        if nonzero:
            self._zeros_left = self._zero_frames
        elif self._zeros_left > 0:
            self._zeros_left -= 1              # 松杆补零阶段

        publish = self._enable and (nonzero or self._zeros_left > 0 or warm)
        if not publish:
            if self._active:
                self._active = False
                self.get_logger().info("摇杆已回中，停止发送行走指令")
            return
        if not self._active and nonzero:
            self._active = True
            self.get_logger().info(
                f"开始发送行走指令 fwd={cmd[0]:.2f} lat={cmd[1]:.2f} "
                f"ang={cmd[2]:.2f}（+lat=左移 +ang=左转）")
        self._pub.publish(Float32MultiArray(data=list(cmd)))


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
