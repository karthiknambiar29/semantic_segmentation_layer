#!/usr/bin/env python3
# Copyright 2026 robot.com
"""Publish synthetic TF + LiDAR + segmentation mask + camera_info to exercise
the semantic_segmentation_layer end to end, with no robot or bag.

    ros2 launch semantic_segmentation_layer segmentation_layer_costmap.launch.py \\
        publish_fake_inputs:=true

The mask is a left->right score gradient (0..255), matching the continuous
zeta_seg / qwen output. With the example config's thresholds the left third
projects as obstacle (lethal), the right third as free, the middle is ignored.
The LiDAR is a flat grid of points in front of the camera.
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

W = H = 200
FX = FY = 100.0
CX = CY = 100.0


class Fake(Node):
    def __init__(self):
        super().__init__("fake_inputs")
        self.tfb = TransformBroadcaster(self)
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.img_pub = self.create_publisher(Image, "/segmentation_mask/image_raw", sensor_qos)
        self.ci_pub = self.create_publisher(CameraInfo, "/segmentation_mask/camera_info", 5)
        self.pc_pub = self.create_publisher(PointCloud2, "/lidar/points", sensor_qos)
        self.timer = self.create_timer(0.1, self.tick)
        self.n = 0

    def tick(self):
        now = self.get_clock().now().to_msg()
        for parent, child in (("odom", "base_link"),
                              ("base_link", "rgb_camera_frame"),
                              ("base_link", "lidar_frame")):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)

        # mask: left->right score gradient 0..255 (continuous, like zeta_seg)
        img = Image()
        img.header.stamp = now
        img.header.frame_id = "rgb_camera_frame"
        img.height, img.width, img.encoding, img.step = H, W, "mono8", W
        row = bytes((x * 255 // (W - 1)) for x in range(W))
        img.data = row * H
        self.img_pub.publish(img)

        ci = CameraInfo()
        ci.header = img.header
        ci.height, ci.width = H, W
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        ci.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.ci_pub.publish(ci)

        # LiDAR: a grid of points 1-4 m in front (physical +x), spread in y/z
        pts = []
        for i in range(30):
            for j in range(20):
                x = 1.0 + 0.1 * i
                y = -1.0 + 0.1 * j
                z = 0.0
                pts.append((x, y, z))
        pc = PointCloud2()
        pc.header.stamp = now
        pc.header.frame_id = "lidar_frame"
        pc.height = 1
        pc.width = len(pts)
        pc.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                     for n, o in (("x", 0), ("y", 4), ("z", 8))]
        pc.is_bigendian = False
        pc.point_step = 12
        pc.row_step = 12 * len(pts)
        pc.is_dense = True
        pc.data = b"".join(struct.pack("<fff", *p) for p in pts)
        self.pc_pub.publish(pc)

        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(f"published {self.n} frames ({len(pts)} pts)")


def main():
    rclpy.init()
    rclpy.spin(Fake())


if __name__ == "__main__":
    main()
