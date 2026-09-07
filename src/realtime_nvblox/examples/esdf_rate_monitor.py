#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2


class Monitor(Node):
    def __init__(self):
        super().__init__('realtime_nvblox_esdf_rate_monitor')
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.n = 0
        self.last_n = 0
        self.last_t = time.monotonic()
        self.create_subscription(PointCloud2, '/realtime_nvblox/esdf_3d', self.cb, qos)
        self.create_timer(1.0, self.report)

    def cb(self, msg):
        self.n += 1

    def report(self):
        now = time.monotonic()
        dt = now - self.last_t
        print(f'esdf_3d: {(self.n-self.last_n)/max(dt,1e-9):.2f} Hz')
        self.last_n = self.n
        self.last_t = now


def main():
    rclpy.init()
    n = Monitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
