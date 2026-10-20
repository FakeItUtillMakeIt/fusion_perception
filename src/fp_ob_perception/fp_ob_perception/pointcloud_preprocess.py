#!/usr/bin/env python3
"""点云预处理：原始点云 -> base_link 系 -> 地面/非地面分离。

V1 地面方案：平地 z 带法（base 系地面高度已知 -wheel_r），
linefit 作为斜坡场景升级位（见方案 §4-1），接口已按可替换设计。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros


def parse_cloud(msg):
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    pts = np.stack([raw[:, off['x']:off['x'] + 4].copy().view(np.float32).ravel(),
                    raw[:, off['y']:off['y'] + 4].copy().view(np.float32).ravel(),
                    raw[:, off['z']:off['z'] + 4].copy().view(np.float32).ravel()], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def make_cloud(header, pts):
    m = PointCloud2()
    m.header = header
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(('x', 'y', 'z'))]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * m.width
    m.data = np.asarray(pts, dtype=np.float32).tobytes()
    m.is_dense = True
    return m


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PreprocessNode(Node):
    def __init__(self):
        super().__init__('pointcloud_preprocess')
        d = self.declare_parameter
        self.in_topic = d('input_topic', '/lidar/points_raw').value
        self.max_range = d('max_range', 20.0).value
        self.min_range = d('min_range', 0.5).value
        self.ground_z = d('ground_z_base', -0.075).value
        self.ground_band = d('ground_band', 0.04).value
        self.max_z = d('max_z', 2.5).value
        self.tf_parent = d('tf_parent', 'base_link').value

        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.pub_ng = self.create_publisher(PointCloud2, '/fp_ob/points_non_ground', 10)
        self.pub_g = self.create_publisher(PointCloud2, '/fp_ob/points_ground', 10)
        self.create_subscription(PointCloud2, self.in_topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'preprocess up: {self.in_topic} -> /fp_ob/points_non_ground')

    def cb(self, msg):
        try:
            tfm = self.buf.lookup_transform(self.tf_parent, msg.header.frame_id,
                                            msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return
        pts = parse_cloud(msg)
        if len(pts) == 0:
            return
        R = quat_to_rot(tfm.transform.rotation)
        t = np.array([tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z])
        p = pts @ R.T + t

        r = np.linalg.norm(p[:, :2], axis=1)
        keep = (r > self.min_range) & (r < self.max_range) & (p[:, 2] < self.max_z)
        p = p[keep]
        if len(p) == 0:
            return
        ground = p[:, 2] < self.ground_z + self.ground_band
        from std_msgs.msg import Header
        hdr = Header(stamp=msg.header.stamp, frame_id=self.tf_parent)
        self.pub_ng.publish(make_cloud(hdr, p[~ground]))
        self.pub_g.publish(make_cloud(hdr, p[ground]))


def main():
    rclpy.init()
    n = PreprocessNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
