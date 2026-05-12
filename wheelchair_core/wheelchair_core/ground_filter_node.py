#!/usr/bin/env python3
"""
ground_filter_node.py

Subscribes to the two raw Livox PointCloud2 topics, removes the ground plane
using RANSAC plane segmentation, and republishes filtered clouds to /cloud_right
and /cloud_left.

Falls back to republishing the original cloud unchanged if RANSAC fails.
"""

import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg


class GroundFilterNode(Node):

    def __init__(self):
        super().__init__('ground_filter_node')

        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('min_inlier_ratio', 0.15)
        self.declare_parameter('normal_z_min', 0.75)

        self.dist_thresh      = self.get_parameter('distance_threshold').value
        self.max_iter         = self.get_parameter('max_iterations').value
        self.min_inlier_ratio = self.get_parameter('min_inlier_ratio').value
        self.normal_z_min     = self.get_parameter('normal_z_min').value

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_right = self.create_publisher(PointCloud2, '/cloud_right', 10)
        self.pub_left  = self.create_publisher(PointCloud2, '/cloud_left',  10)

        self.create_subscription(
            PointCloud2, '/livox/lidar_192_168_1_105',
            self.cb_right, sub_qos)
        self.create_subscription(
            PointCloud2, '/livox/lidar_192_168_1_172',
            self.cb_left, sub_qos)

        self.get_logger().info(
            f'Ground filter ready | dist_thresh={self.dist_thresh}m '
            f'iterations={self.max_iter} normal_z_min={self.normal_z_min}')

    # ── point extraction ──────────────────────────────────────────────────────

    def _extract_xyz(self, msg: PointCloud2):
        """Extract XYZ as float32 Nx3 array from any PointCloud2."""
        # Find x, y, z field offsets
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            return None

        fx, fy, fz = fields['x'], fields['y'], fields['z']

        # Only handle FLOAT32 (type=7)
        if not all(f.datatype == PointField.FLOAT32 for f in (fx, fy, fz)):
            return None

        point_step = msg.point_step
        n = msg.width * msg.height
        data = bytes(msg.data)

        xyz = np.frombuffer(data, dtype=np.uint8).reshape(n, point_step)
        x = np.frombuffer(xyz[:, fx.offset:fx.offset+4].tobytes(), dtype=np.float32)
        y = np.frombuffer(xyz[:, fy.offset:fy.offset+4].tobytes(), dtype=np.float32)
        z = np.frombuffer(xyz[:, fz.offset:fz.offset+4].tobytes(), dtype=np.float32)

        pts = np.column_stack([x, y, z])

        # Remove NaN/Inf
        valid = np.isfinite(pts).all(axis=1)
        return pts[valid]

    # ── RANSAC ────────────────────────────────────────────────────────────────

    def _non_ground_mask(self, xyz: np.ndarray) -> np.ndarray:
        """Return boolean mask: True = keep (not ground)."""
        n = len(xyz)
        if n < 3:
            return np.ones(n, dtype=bool)

        best_inliers = np.zeros(n, dtype=bool)
        best_count   = 0

        for _ in range(self.max_iter):
            idx = np.random.choice(n, 3, replace=False)
            p1, p2, p3 = xyz[idx[0]], xyz[idx[1]], xyz[idx[2]]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-6:
                continue
            normal /= norm_len

            if abs(normal[2]) < self.normal_z_min:
                continue

            dist = np.abs((xyz - p1) @ normal)
            inliers = dist < self.dist_thresh
            count   = int(inliers.sum())

            if count > best_count:
                best_count   = count
                best_inliers = inliers

        if best_count < self.min_inlier_ratio * n:
            return np.ones(n, dtype=bool)

        return ~best_inliers

    # ── cloud creation ────────────────────────────────────────────────────────

    def _make_xyz32_cloud(self, header, xyz: np.ndarray) -> PointCloud2:
        """Create a PointCloud2 with XYZ float32 fields."""
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        data = xyz.astype(np.float32).tobytes()

        msg = PointCloud2()
        msg.header      = header
        msg.height      = 1
        msg.width       = len(xyz)
        msg.fields      = fields
        msg.is_bigendian = False
        msg.point_step  = point_step
        msg.row_step    = point_step * len(xyz)
        msg.data        = data
        msg.is_dense    = True
        return msg

    # ── filter pipeline ───────────────────────────────────────────────────────

    def _filter(self, msg: PointCloud2) -> PointCloud2:
        try:
            xyz = self._extract_xyz(msg)
            if xyz is None or len(xyz) == 0:
                return msg

            mask = self._non_ground_mask(xyz)
            kept = xyz[mask]

            return self._make_xyz32_cloud(msg.header, kept)
        except Exception as e:
            self.get_logger().warn(f'Ground filter error: {e}', throttle_duration_sec=5.0)
            return msg

    def cb_right(self, msg: PointCloud2):
        try:
            self.pub_right.publish(self._filter(msg))
        except Exception as e:
            self.get_logger().error(f'cb_right failed: {e}')

    def cb_left(self, msg: PointCloud2):
        try:
            self.pub_left.publish(self._filter(msg))
        except Exception as e:
            self.get_logger().error(f'cb_left failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GroundFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
