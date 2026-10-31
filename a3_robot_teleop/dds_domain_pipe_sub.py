#!/usr/bin/env python3
"""域间管道-发送端：在本进程所在 ROS 域订阅话题，CDR 序列化后经本机 TCP 发给接收端。

rclpy 单进程只能属于一个 ROS 域，跨域桥接 = 两个进程 + 本机 TCP 串联。
本脚本只做"订阅 + 转发"，与 dds_domain_pipe_pub.py 成对使用：
订阅回调里把消息 serialize 成 CDR 字节， length-prefix 后写 TCP；
连接断开自动重连，未连接期间丢弃新帧（100Hz 状态流丢一帧无害）。

用法（ROS_DOMAIN_ID 由外部指定为【源域】）：
  ROS_DOMAIN_ID=232 python3 dds_domain_pipe_sub.py \
      --topic /motion/control/arm_joint_state \
      --type sensor_msgs/msg/JointState --port 53200 --qos best_effort
  ROS_DOMAIN_ID=42  python3 dds_domain_pipe_sub.py \
      --topic /motion/control/arm_joint_command \
      --type sensor_msgs/msg/JointState --port 53201 --qos reliable

只测通信、不触动作：订阅是只读的；真正会驱动机器人的是指令方向的管道，
那把"什么时候起"交给操作者决定。
"""
import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


class PipeSub(Node):
    def __init__(self, topic, msg_type_name, port, reliable):
        super().__init__(f"pipe_sub_{topic.strip('/').replace('/', '_')}")
        self.msg_type = get_message(msg_type_name)
        self.topic = topic
        self.port = port
        self.sock = None
        self.sock_lock = threading.Lock()
        self.sent = 0
        self.dropped = 0

        # QoS 按方向区分（桥接两侧发布者策略可能不同，不匹配会一帧都收不到）：
        # 状态方向（订阅 MC）：MC 用 BEST_EFFORT 发布 100Hz 流，必须 BEST_EFFORT 订阅；
        # 指令方向（订阅主机）：主机发布 RELIABLE，跨 WiFi 必须 RELIABLE 订阅防丢帧
        # （MC 要求相邻指令间隔 ≤30ms，尽力模式在弱网下丢帧会直接造成卡顿）。
        policy = (ReliabilityPolicy.RELIABLE if reliable
                  else ReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(depth=50, reliability=policy)
        self.create_subscription(self.msg_type, topic, self.on_msg, qos)
        self.conn_thread = threading.Thread(
            target=self._conn_loop, daemon=True)
        self.conn_thread.start()
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"pipe_sub 就绪：域内订阅 {topic} ({msg_type_name}) -> "
            f"TCP 127.0.0.1:{port}")

    def _conn_loop(self):
        while rclpy.ok():
            try:
                s = socket.create_connection(("127.0.0.1", self.port), 2.0)
                # create_connection 的超时会留在 socket 上：recv 2s 无数据就
                # 抛 TimeoutError，被当成断线 -> 连接每 2.5s 抖一次、抖窗口内
                # 的帧全丢（本地实测 1000 帧丢 300+）。必须恢复阻塞模式。
                s.settimeout(None)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.sock_lock:
                    self.sock = s
                self.get_logger().info(f"已连接 127.0.0.1:{self.port}")
                # 单向流：数据全走 sendall，对端不发任何东西。
                # 不读 socket；对端消失时下一次 sendall 会失败再重连。
                while rclpy.ok() and self.sock is s:
                    time.sleep(0.5)
            except OSError:
                pass
            finally:
                with self.sock_lock:
                    if self.sock is not None:
                        try:
                            self.sock.close()
                        except OSError:
                            pass
                        self.sock = None
            time.sleep(0.5)

    def on_msg(self, msg):
        payload = serialize_message(msg)
        frame = struct.pack(">I", len(payload)) + payload
        with self.sock_lock:
            if self.sock is None:
                self.dropped += 1
                return
            try:
                self.sock.sendall(frame)
                self.sent += 1
            except OSError:
                self.dropped += 1
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None

    def report(self):
        self.get_logger().info(
            f"转发 {self.sent} 帧，未连接时丢弃 {self.dropped} 帧")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--type", required=True, dest="msg_type")
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable")
    args = ap.parse_args()

    rclpy.init()
    node = PipeSub(args.topic, args.msg_type, args.port,
                   reliable=args.qos == "reliable")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
