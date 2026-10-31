#!/usr/bin/env python3
"""域间管道-接收端：监听本机 TCP，把 CDR 字节反序列化后在【本进程域】重发话题。

与 dds_domain_pipe_sub.py 成对使用。TCP 服务端，单客户端，断开后等下一次接入。
消息类型必须与发送端一致（--type 相同），时间戳等字段原样保留
（在主机侧会被 mc_state_to_joint_states 用主机时钟重打，不受影响）。

用法（ROS_DOMAIN_ID 由外部指定为【目标域】）：
  ROS_DOMAIN_ID=42 python3 dds_domain_pipe_pub.py \
      --topic /motion/control/arm_joint_state \
      --type sensor_msgs/msg/JointState --port 53200
"""
import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class PipePub(Node):
    def __init__(self, topic, msg_type_name, port):
        super().__init__(f"pipe_pub_{topic.strip('/').replace('/', '_')}")
        self.msg_type = get_message(msg_type_name)
        self.pub = self.create_publisher(self.msg_type, topic, 50)
        self.port = port
        self.recv_count = 0
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", port))
        self.server.listen(1)
        self.stop = False
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"pipe_pub 就绪：TCP 127.0.0.1:{port} -> 域内发布 {topic} "
            f"({msg_type_name})")

    def _serve(self):
        while not self.stop:
            try:
                conn, _ = self.server.accept()
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info("发送端已接入")
            try:
                self._pump(conn)
            except OSError:
                pass
            finally:
                try:
                    conn.close()
                except OSError:
                    pass

    def _pump(self, conn):
        buf = b""
        while not self.stop:
            chunk = conn.recv(65536)
            if not chunk:
                return
            buf += chunk
            while len(buf) >= 4:
                (n,) = struct.unpack(">I", buf[:4])
                if len(buf) < 4 + n:
                    break
                payload = buf[4:4 + n]
                buf = buf[4 + n:]
                msg = deserialize_message(payload, self.msg_type)
                self.pub.publish(msg)
                self.recv_count += 1

    def report(self):
        self.get_logger().info(f"已重发 {self.recv_count} 帧")

    def destroy_node(self):
        self.stop = True
        try:
            self.server.close()
        except OSError:
            pass
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--type", required=True, dest="msg_type")
    ap.add_argument("--port", type=int, required=True)
    # 与发送端参数对齐用；发布侧始终 RELIABLE（主机订阅端默认 Reliable，
    # 尽力发布会与订阅端不兼容），本参数仅接受并忽略。
    ap.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable")
    args = ap.parse_args()

    rclpy.init()
    node = PipePub(args.topic, args.msg_type, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
