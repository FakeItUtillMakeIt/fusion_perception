import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformListener

from fp_common.pointcloud import read_xyz_label
from fp_common.policy import Policy
from fp_common.transforms import load_calibration

PASSABLE, SOFT, HARD, TRANSP, UNKNOWN, NFREE = 0, 1, 2, 3, 4, 5


def quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


class TraversabilityGridNode(Node):
    def __init__(self):
        super().__init__('fp_traversability_grid')
        self.declare_parameter('labeled_topic', '/fp/labeled_points')
        self.declare_parameter('policy_path', '')
        self.declare_parameter('grid_frame', 'odom')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('extent', 10.0)
        self.declare_parameter('decay_tau', 2.0)
        self.declare_parameter('min_z', -0.7)
        self.declare_parameter('max_z', 2.5)
        self.declare_parameter('soft_cost', 50)
        self.declare_parameter('hard_cost', 90)
        self.declare_parameter('transparent_cost', 98)
        self.declare_parameter('unknown_cost', 65)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('grid_topic', '/fp/traversability')
        self.declare_parameter('vis_topic', '/fp/traversability_vis')
        self.declare_parameter('anchor_bias_xy', [0.0, 0.0])
        # camera blind-spot fill: ray-cast hard/transparent label pixels to ground plane
        self.declare_parameter('fill_labels', False)
        self.declare_parameter('labels_topic', '/fp/labels')
        self.declare_parameter('calib_path', '')
        self.declare_parameter('fill_range', 12.0)
        self.declare_parameter('fill_stride', 4)
        self.declare_parameter('fill_weight', 1.5)

        self.res = float(self.get_parameter('resolution').value)
        self.n = int(self.get_parameter('extent').value / self.res)
        self.ev = np.zeros((self.n, self.n, NFREE), np.float32)
        self.fill = np.zeros((self.n, self.n), np.float32)   # persistent camera-fill (no decay)
        self.anchor = None
        self.last_t = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.policy = None
        ppath = self.get_parameter('policy_path').value
        if ppath:
            self.policy = Policy(ppath)

        self.calib = None
        cpath = self.get_parameter('calib_path').value
        if self.get_parameter('fill_labels').value and cpath:
            self.calib = load_calibration(cpath)
            self.Kinv = np.linalg.inv(self.calib['K'])

        self.pub_grid = self.create_publisher(OccupancyGrid, self.get_parameter('grid_topic').value, 10)
        self.pub_vis = self.create_publisher(Image, self.get_parameter('vis_topic').value, 10)
        self.create_subscription(PointCloud2, self.get_parameter('labeled_topic').value,
                                 self._cloud_cb, 10)
        if self.calib is not None:
            self.create_subscription(Image, self.get_parameter('labels_topic').value,
                                     self._labels_cb, 10)
        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / hz, self._publish)

    def _group_of(self, labels):
        g = np.full(len(labels), UNKNOWN, np.int8)
        if self.policy is not None:
            gi = self.policy.group_lut[labels]
            name2idx = {'passable': PASSABLE, 'soft': SOFT, 'hard': HARD, 'transparent': TRANSP}
            for gname, idx in name2idx.items():
                if gname in self.policy.group_index:
                    g[gi == self.policy.group_index[gname]] = idx
        g[labels == 255] = UNKNOWN
        return g

    def _cloud_cb(self, msg):
        try:
            tfm = self.tf_buffer.lookup_transform(self.get_parameter('grid_frame').value,
                                                  msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'tf {self.get_parameter("grid_frame").value}<-{msg.header.frame_id}: {e}',
                                   throttle_duration_sec=5.0)
            return
        xyz, labels = read_xyz_label(msg)
        if len(xyz) == 0:
            return
        R = quat_to_R(tfm.transform.rotation.x, tfm.transform.rotation.y,
                      tfm.transform.rotation.z, tfm.transform.rotation.w)
        t = np.array([tfm.transform.translation.x, tfm.transform.translation.y,
                      tfm.transform.translation.z])
        pts = xyz @ R.T + t
        zmin = float(self.get_parameter('min_z').value) + t[2]
        zmax = float(self.get_parameter('max_z').value) + t[2]
        keep = (pts[:, 2] > zmin) & (pts[:, 2] < zmax)
        pts, labels = pts[keep], labels[keep]
        if len(pts) == 0:
            return
        groups = self._group_of(labels)

        if self.anchor is None:
            bias = np.array([float(v) for v in self.get_parameter('anchor_bias_xy').value],
                            dtype=np.float64)
            self.anchor = t[:2] - np.array([self.n * self.res / 2, self.n * self.res / 2]) + bias
        now = time.time()
        if self.last_t is not None:
            self.ev *= math.exp(-(now - self.last_t) / float(self.get_parameter('decay_tau').value))
        self.last_t = now

        ij = ((pts[:, :2] - self.anchor) / self.res).astype(np.int32)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.n) & (ij[:, 1] >= 0) & (ij[:, 1] < self.n)
        ij, groups = ij[ok], groups[ok]
        acc = self.ev.reshape(-1, NFREE)
        for k in range(NFREE):
            idx = ij[:, 1] * self.n + ij[:, 0]
            np.add.at(acc[:, k], idx, (groups == k).astype(np.float32))

    def _labels_cb(self, msg):
        if self.anchor is None or msg.encoding != 'mono8':
            return
        try:
            tfm = self.tf_buffer.lookup_transform(self.get_parameter('grid_frame').value,
                                                  msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        labels = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
        groups = self._group_of(labels.ravel()).reshape(msg.height, msg.width)
        bad = np.isin(groups, [HARD, TRANSP])
        # per image column, keep only the LOWEST hard pixel (wall-ground contact line)
        # to avoid smearing tall walls far behind their base
        H, W = bad.shape
        rows = np.arange(H)[:, None]
        masked = np.where(bad, rows, -1)
        bottom = masked.max(axis=0)          # (W,) lowest hard row per column, -1 none
        cols = np.where(bottom >= 0)[0]
        if len(cols) == 0:
            return
        stride = int(self.get_parameter('fill_stride').value)
        cols = cols[::stride]
        vs = bottom[cols].astype(np.float64)
        us = cols.astype(np.float64)
        R = quat_to_R(tfm.transform.rotation.x, tfm.transform.rotation.y,
                      tfm.transform.rotation.z, tfm.transform.rotation.w)
        o = np.array([tfm.transform.translation.x, tfm.transform.translation.y,
                      tfm.transform.translation.z])
        pix = np.stack([us, vs, np.ones(len(us))], axis=1)
        dcam = pix @ self.Kinv.T
        dcam /= np.linalg.norm(dcam, axis=1, keepdims=True)
        dmap = dcam @ R.T
        with np.errstate(divide='ignore', invalid='ignore'):
            tp = -o[2] / dmap[:, 2]
        ok = (dmap[:, 2] < 0) & (tp > 1.5) & (tp < float(self.get_parameter('fill_range').value))
        pts = o[None] + dmap * tp[:, None]
        ij = ((pts[:, :2] - self.anchor) / self.res).astype(np.int32)
        ok &= (ij[:, 0] >= 0) & (ij[:, 0] < self.n) & (ij[:, 1] >= 0) & (ij[:, 1] < self.n)
        ij = ij[ok]
        # persistent accumulation: each observation reinforces, never decays
        np.add.at(self.fill, (ij[:, 1], ij[:, 0]), 1.0)

    def _cost_grid(self):
        obs = self.ev[..., [SOFT, HARD, TRANSP]]
        # safety priority: transparent > hard > soft on ties
        bonus = np.array([0.0, 0.5, 1.0], np.float32)
        obs_w = obs + bonus[None, None, :] * (obs > 0.5) * 1e-3
        max_obs = obs_w.max(-1)
        arg_obs = obs_w.argmax(-1)
        pass_e = self.ev[..., PASSABLE]
        unk_e = self.ev[..., UNKNOWN]
        cost = np.full(self.ev.shape[:2], -1, np.int8)
        has_obs = (max_obs > 0.5) & (max_obs > pass_e)
        # arg_obs is position in [SOFT, HARD, TRANSP] -> 0/1/2
        pos_cost = np.array([int(self.get_parameter('soft_cost').value),
                             int(self.get_parameter('hard_cost').value),
                             int(self.get_parameter('transparent_cost').value)], np.int8)
        cost[has_obs] = pos_cost[arg_obs[has_obs]]
        cost[~has_obs & (pass_e > 0.5)] = 0
        rest = ~has_obs & (pass_e <= 0.5) & (unk_e > 0.5)
        cost[rest] = int(self.get_parameter('unknown_cost').value)
        # persistent camera-fill: force hard where the camera saw an obstacle (glass blind-spot)
        fh = self.fill >= 1.0
        cost[fh] = int(self.get_parameter('hard_cost').value)
        return cost

    def _publish(self):
        if self.anchor is None:
            return
        cost = self._cost_grid()
        m = OccupancyGrid()
        m.header.frame_id = self.get_parameter('grid_frame').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = self.res
        m.info.width = self.n
        m.info.height = self.n
        m.info.origin.position.x = float(self.anchor[0])
        m.info.origin.position.y = float(self.anchor[1])
        m.info.origin.orientation.w = 1.0   # valid identity quat (default 0,0,0,0 is invalid -> RViz renders the map rotated)
        m.data = cost.reshape(-1).tolist()
        self.pub_grid.publish(m)

        lut = np.array([[40, 40, 40], [0, 200, 0], [0, 220, 220], [0, 0, 230],
                        [255, 0, 255], [60, 160, 255]], np.uint8)
        img = np.zeros((self.n, self.n, 3), np.uint8)
        img[cost == -1] = lut[0]
        img[cost == 0] = lut[1]
        img[(cost > 0) & (cost < 60)] = lut[2]
        img[(cost >= 60) & (cost < 90)] = lut[5]
        img[(cost >= 90) & (cost < 98)] = lut[3]
        img[cost >= 98] = lut[4]
        vis = Image()
        vis.header = m.header
        vis.encoding = 'bgr8'
        vis.height = self.n
        vis.width = self.n
        vis.step = self.n * 3
        vis.data = np.ascontiguousarray(img[::-1]).tobytes()
        self.pub_vis.publish(vis)


def main():
    rclpy.init()
    node = TraversabilityGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
