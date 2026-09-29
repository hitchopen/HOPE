#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from audio_msgs.srv import RequestAudioFocus

class AudioFocusClient(Node):
def **init**(self):
super().**init**('audio_focus_client')

```
    self.client = self.create_client(
        RequestAudioFocus,
        '/audio_5Fmsgs/srv/AbandonAudioFocus'
    )

    # 等待 service 出现
    while not self.client.wait_for_service(timeout_sec=2.0):
        self.get_logger().warn('Service not available, waiting...')

def send_request(self):
    req = RequestAudioFocus.Request()

    req.focus_requester.pkg_name = "audio_examples_sender"
    req.focus_requester.priority = 6
    req.focus_requester.priority_weight = 1

    future = self.client.call_async(req)
    rclpy.spin_until_future_complete(self, future)

    if future.result() is not None:
        self.get_logger().info(f"Response: {future.result()}")
    else:
        self.get_logger().error("Service call failed")
```

def main():
rclpy.init()
node = AudioFocusClient()
node.send_request()
node.destroy_node()
rclpy.shutdown()

if **name** == '**main**':
main()

