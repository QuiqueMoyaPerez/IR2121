#!/usr/bin/env python3
"""Reenvia /scan como /scan_fixed con SIEMPRE 360 lecturas (el LIDAR real da 276-283)."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)
from sensor_msgs.msg import LaserScan

N = 360


class ScanFixer(Node):
    def __init__(self):
        super().__init__("scan_fixer")
        qos_out = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=50)
        self.pub = self.create_publisher(LaserScan, "/scan_fixed", qos_out)
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)
        self.inc = 2.0 * math.pi / N
        self.ang = np.arange(N) * self.inc
        self.get_logger().info("scan_fixer: /scan -> /scan_fixed con %d lecturas" % N)

    def cb(self, m):
        n = len(m.ranges)
        if n < 2 or m.angle_increment == 0.0:
            return
        r = np.asarray(m.ranges, dtype=np.float32)
        rel = np.mod(self.ang - m.angle_min, 2.0 * math.pi)
        idx = np.rint(rel / abs(m.angle_increment)).astype(int)
        out = np.full(N, np.inf, dtype=np.float32)
        ok = idx < n
        out[ok] = r[idx[ok]]
        o = LaserScan()
        o.header = m.header
        o.angle_min = 0.0
        o.angle_max = (N - 1) * self.inc
        o.angle_increment = self.inc
        o.time_increment = m.time_increment * n / N
        o.scan_time = m.scan_time
        o.range_min = m.range_min
        o.range_max = m.range_max
        o.ranges = out.tolist()
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(ScanFixer())


if __name__ == "__main__":
    main()
