#!/usr/bin/env python3
"""玻璃/透明障碍检测器 —— 激光优先、语义补盲（负障碍融合）。

优先级仲裁（激光为主）：
  1) negative: 语义=墙 的像素区，其下沿比该列"实体激光回波"最低行还低 gap_px 以上
     => 该区域激光无回波但相机看到"墙" => 透明面板。取墙像素底边为地脚线。
  2) glass:    语义直接命中玻璃类(windowpane/glass)且面积足够 => 底边地脚线（辅助）。
  3) roi:      /fp_sem/manual_roi 手动框（调试兜底）。
地脚线像素经标定 K/R/t 反投影到地面 -> base_link 玻璃点云/多边形 -> costmap 观测源。

输入: /fp/labels(mono8), /lidar/points_raw, /fp_sem/manual_roi
输出: /fp_sem/glass_cloud(PointCloud2 base_link), /fp_sem/glass_obstacles, /fp_sem/glass_markers
"""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Point, Polygon, PolygonStamped
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from fp_ob_msgs.msg import Obstacle, ObstacleArray
from fp_common.transforms import load_calibration, project_points


def make_cloud(stamp, pts):
    m = PointCloud2()
    m.header = Header(stamp=stamp, frame_id='base_link')
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]
    m.point_step = 12
    m.row_step = 12 * m.width
    m.data = np.asarray(pts, dtype=np.float32).tobytes() if len(pts) else b''
    m.is_dense = True
    return m


class GlassDetector(Node):
    def __init__(self):
        super().__init__('glass_detector')
        d = self.declare_parameter
        self.glass_ids = list(d('glass_ids', [8, 147]).value)
        self.wall_ids = list(d('wall_ids', [0]).value)
        self.negative = d('negative_mode', True).value
        self.gap_px = int(d('gap_px', 20).value)
        self.min_area = d('min_area_px', 2000).value
        self.pt_z_min = d('nonground_z_min', 0.10).value
        self.ground_z_lidar = d('ground_z_lidar', -0.525).value
        self.z_rows = list(d('curtain_z', [-0.05, 0.05, 0.15, 0.3, 0.5]).value)
        self.densify = d('densify_m', 0.1).value
        self.hold_sec = d('hold_sec', 3.0).value
        self.roi_hold = d('roi_hold_sec', 30.0).value
        self.max_range = d('max_range_m', 10.0).value
        calib = d('calib', '').value
        if not calib or not os.path.exists(calib):
            from ament_index_python.packages import get_package_share_directory
            calib = os.path.join(get_package_share_directory('fp_semantic_nav'),
                                 'config', 'sim_truth_robot1.yaml')
        c = load_calibration(calib)
        self.K, self.R, self.t = c['K'], c['R'], c['t']
        self.K_inv = np.linalg.inv(self.K)
        self.c3 = self.R[:, 2]

        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.pub_cloud = self.create_publisher(PointCloud2, '/fp_sem/glass_cloud', 10)
        self.pub_obs = self.create_publisher(ObstacleArray, '/fp_sem/glass_obstacles', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/fp_sem/glass_markers', 10)
        self.create_subscription(Image, '/fp/labels', self.cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/points_raw', self.pts_cb, qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/fp_sem/manual_roi', self.roi_cb, 10)
        self.raw_pts = None
        self.roi_line, self.roi_stamp = None, None
        self.last_line, self.last_stamp, self.last_src = None, None, ''
        self.get_logger().info('glass_detector up (lidar-first, semantics-fills-blind): '
                               'negative=%s wall_ids=%s glass_ids=%s' % (self.negative, self.wall_ids, self.glass_ids))

    def pts_cb(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        off = {f.name: f.offset for f in msg.fields}
        p = np.stack([raw[:, off[a]:off[a] + 4].copy().view(np.float32).ravel() for a in 'xyz'], 1)
        pb = p + np.array([0.25, 0.0, 0.525])
        self.raw_pts = pb[pb[:, 2] > self.pt_z_min]

    def roi_cb(self, msg):
        pts = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        if len(pts) < 3:
            return
        us = [p[0] for p in pts]
        vs = [p[1] for p in pts]
        cols = np.arange(min(us), max(us), 2.0)
        self.roi_line = np.stack([cols, np.full_like(cols, max(vs) + 2.0)], 1)
        self.roi_stamp = self.get_clock().now().nanoseconds
        self.get_logger().info('manual roi accepted')

    def solid_occ_rows(self, W, H):
        # 实体激光点投影 -> 每列最低像素行（激光有回波的证据）
        pl = self.raw_pts[:, :3] - np.array([0.25, 0.0, 0.525])
        uv, depth, _ = project_points(pl, self.K, self.R, self.t)
        ok = (depth > 0.3) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
        occ = {}
        for u, v in uv[ok].astype(int):
            for du in range(-3, 4):
                c2 = u + du
                if 0 <= c2 < W:
                    occ[c2] = max(occ.get(c2, -1), int(v))
        return occ

    def largest_blob(self, mask):
        n, lmap, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        best, barea = -1, 0
        for i in range(1, n):
            a = stats[i, cv2.CC_STAT_AREA]
            if a > barea:
                best, barea = i, a
        if best < 0 or barea < self.min_area:
            return None
        return lmap == best

    def bottom_edge(self, blob):
        cols = np.where(blob.any(axis=0))[0]
        return np.stack([cols.astype(np.float64),
                         np.array([np.where(blob[:, c])[0].max() for c in cols], np.float64) + 2.0], 1)

    def negative_line(self, blob, occ, H, W):
        cols = np.where(blob.any(axis=0))[0]
        cand = []
        for c in cols:
            vb = int(np.where(blob[:, c])[0].max())
            vp = occ.get(int(c), -1)
            if vb - vp >= self.gap_px and vb > H * 0.35:
                cand.append((c, vb))
        if len(cand) < max(10, len(cols) * 0.3):
            return None
        return np.array(cand, dtype=np.float64)

    def cb(self, msg):
        lab = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        H, W = lab.shape
        line, src = None, ''
        # 1) 激光优先：只在"实体激光无回波"的墙像素区用语义
        if self.negative and self.raw_pts is not None and len(self.raw_pts) >= 10:
            wm = np.isin(lab, self.wall_ids).astype(np.uint8)
            k = np.ones((5, 5), np.uint8)
            wm = cv2.morphologyEx(wm, cv2.MORPH_OPEN, k)
            wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, k)
            blob = self.largest_blob(wm)
            if blob is not None:
                line = self.negative_line(blob, self.solid_occ_rows(W, H), H, W)
                src = 'negative'
        # 2) 玻璃类直检（辅助）
        if line is None:
            gm = np.isin(lab, self.glass_ids).astype(np.uint8)
            gm = cv2.morphologyEx(gm, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            blob = self.largest_blob(gm)
            if blob is not None:
                line, src = self.bottom_edge(blob), 'glass'
        # 3) 手动兜底
        if line is None and self.roi_line is not None and \
           abs((msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec - self.roi_stamp) * 1e-9) < self.roi_hold:
            line, src = self.roi_line, 'roi'
        if line is None:
            self.publish_hold(msg.header.stamp)
            return
        pts = self.ground_points(line)
        if pts is None or len(pts) < 2:
            self.publish_hold(msg.header.stamp)
            return
        self.last_line, self.last_stamp, self.last_src = pts, msg.header.stamp, src
        self.publish(pts, msg.header.stamp, src)

    def ground_points(self, uv):
        hom = np.stack([uv[:, 0], uv[:, 1], np.ones(len(uv))], 1)
        d = hom @ self.K_inv.T
        denom = d @ self.c3
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (self.ground_z_lidar + self.c3 @ self.t) / denom
        ok = (denom < -1e-9) & (s > 0.2) & (s < 40.0)
        p_cam = d[ok] * s[ok][:, None]
        p_lidar = (p_cam - self.t) @ self.R
        try:
            tfm = self.buf.lookup_transform('base_link', 'lidar_link', rclpy.time.Time())
        except Exception:
            return None
        q = tfm.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        Rb = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        tb = np.array([tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z])
        p_base = p_lidar @ Rb.T + tb
        r = np.linalg.norm(p_base[:, :2], axis=1)
        return p_base[r < self.max_range]

    def densify_line(self, pts):
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            L = float(np.hypot(*(b[:2] - a[:2])))
            k = max(int(L / self.densify), 1)
            for i in range(1, k + 1):
                out.append(a + (b - a) * (i / k))
        return np.array(out)

    def publish(self, line, stamp, src=''):
        dense = self.densify_line(line)
        p0, p1 = dense[0][:2], dense[-1][:2]
        dv = p1 - p0
        nrm = np.array([-dv[1], dv[0]])
        nrm = nrm / max(np.linalg.norm(nrm), 1e-6)
        cloud = []
        for x, y in dense[:, :2]:
            for off in (-0.15, 0.0, 0.15):
                for z in self.z_rows:
                    cloud.append((float(x + nrm[0] * off), float(y + nrm[1] * off), float(z)))
        self.pub_cloud.publish(make_cloud(stamp, cloud))

        arr = ObstacleArray()
        arr.header = Header(stamp=stamp, frame_id='base_link')
        ob = Obstacle()
        ob.header = arr.header
        ob.class_name = 'vertical_glass'
        ob.source = 'lidar_blind+semantic' if src == 'negative' else src
        ob.confidence = 0.9 if src == 'negative' else 0.7
        ob.id = 1
        d0, d1 = dense[0][:2], dense[-1][:2]
        nn = np.array([-dv[1], dv[0]]) / max(np.linalg.norm(dv), 1e-6) * 0.12
        corners = [d0 + nn, d1 + nn, d1 - nn, d0 - nn]
        ob.polygon = Polygon(points=[Point32(x=float(p[0]), y=float(p[1]), z=0.0) for p in corners])
        ob.min_height = -0.075
        ob.max_height = 1.2
        arr.obstacles.append(ob)
        self.pub_obs.publish(arr)

        mk = MarkerArray()
        m = Marker()
        m.header = arr.header
        m.ns = 'glass'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.b, m.color.a = 1.0, 1.0, 0.9
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.1) for p in dense[:, :2]]
        mk.markers.append(m)
        t = Marker()
        t.header = arr.header
        t.ns = 'glass'
        t.id = 1
        t.type = Marker.TEXT_VIEW_FACING
        t.pose.position.x = float(dense[:, 0].mean())
        t.pose.position.y = float(dense[:, 1].mean())
        t.pose.position.z = 1.4
        t.scale.z = 0.25
        t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.0, 1.0, 1.0
        t.text = 'vertical_glass[%s]' % src
        mk.markers.append(t)
        self.pub_mk.publish(mk)

    def publish_hold(self, stamp):
        if self.last_line is None:
            return
        dt = abs((stamp.sec * 1e9 + stamp.nanosec -
                  (self.last_stamp.sec * 1e9 + self.last_stamp.nanosec)) * 1e-9)
        if dt > self.hold_sec:
            self.pub_cloud.publish(make_cloud(stamp, []))
            return
        self.publish(self.last_line, stamp, self.last_src)


def main():
    rclpy.init()
    n = GlassDetector()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
