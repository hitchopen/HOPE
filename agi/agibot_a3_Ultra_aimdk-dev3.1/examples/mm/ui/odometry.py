import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import yaml
import threading
import queue
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

SERVER_IP = '192.168.34.136'
SERVER_PORT = 5001


class OdomForwarder(Node):

    def __init__(self):
        super().__init__('odom_forwarder')

        # 队列用于缓存数据（防止阻塞）
        self.msg_queue = queue.Queue(maxsize=100)

        # 启动发送线程（负责socket连接和重连）
        self.sock = None
        self.sender_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.sender_thread.start()

        # 完全匹配 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        topic_name = '/slam/localization/odometry'
        self.sub = self.create_subscription(
            Odometry,
            topic_name,
            self.callback,
            qos
        )
        print("Subscription created.")

        # 用于检测定位消息超时
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        self.timeout_sec = 2.0  # 超时时间（秒）
        self.timer = self.create_timer(0.5, self.check_timeout)
        self.locate_failed_sent = False  # 避免重复插入定位失败

    def check_timeout(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_msg_time > self.timeout_sec:
            if not self.locate_failed_sent:
                fail_msg = yaml.dump({"status": 'locate_failed'})
                try:
                    if self.msg_queue.full():
                        self.msg_queue.get()
                    self.msg_queue.put_nowait(fail_msg)
                    self.locate_failed_sent = True
                except Exception as e:
                    self.get_logger().error(f"Queue error (timeout): {e}")
        else:
            self.locate_failed_sent = False

    def callback(self, msg):
        try:
            # 更新最后收到消息的时间
            self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
            self.locate_failed_sent = False  # 收到消息后允许再次插入失败
            msg_dict = {
                "t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            }

            yaml_str = yaml.dump(msg_dict)

            # 如果队列满，丢弃最旧数据（保持低延迟）
            if self.msg_queue.full():
                self.msg_queue.get()

            self.msg_queue.put_nowait(yaml_str)

        except Exception as e:
            self.get_logger().error(f"Queue error: {e}")

    def send_loop(self):
        while True:
            # 保证socket已连接
            if self.sock is None:
                while True:
                    try:
                        print("Connecting to server...")
                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        sock.settimeout(3)
                        sock.connect((SERVER_IP, SERVER_PORT))
                        sock.settimeout(None)
                        self.sock = sock
                        print("Socket connected.")
                        break
                    except Exception as e:
                        print(f"Socket connect failed: {e}, retrying in 2s...")
                        time.sleep(2)
            try:
                data = self.msg_queue.get()
                self.sock.sendall((data + "\n").encode())
            except Exception as e:
                print("Send error:", e)
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                self.sock = None
                time.sleep(2)


def main():
    rclpy.init()
    node = OdomForwarder()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
