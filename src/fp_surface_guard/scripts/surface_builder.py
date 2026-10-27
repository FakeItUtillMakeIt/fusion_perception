#!/usr/bin/env python3
"""GlassGuard 风格表面构建器：视觉提议候选面，激光种子做几何验证/修正。

与旧 glass_detector 的权威方向相反（存在性由视觉决定，激光无否决权）：
  * 语义掩码直接成立候选 -> 地脚线/地面包络 -> SurfaceArray；
  * 激光只做两件事：a) 种子点修正候选面深度（blob 内回波中位距离 vs 掩码底沿投影距离）
    b) 负候选补盲（墙/天花板类像素但激光无回波证据：仿真面板被误分为 wall/ceiling 的路径）。
输出 polygon(base_link) -> costmap 的 SurfaceOverrideLayer 以"覆盖 free"权限写入。

候选组 -> 输出类:
  glass_ids 直检 | wall_ids 负候选 | 手动ROI   -> 'glass'      (竖直面板, 脚线条带)
  water_ids 直检                               -> 'water'      (贴地面, 掩码凸包)
  stair_ids  直检                              -> 'stair_down' (贴地面, 掩码凸包)
输入: /fp/labels, /lidar/points_raw, /fp_sem/manual_roi(调试)
输出: /fp_guard/surfaces(SurfaceArray), /fp_guard/markers
"""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Point, Polygon, PolygonStamped
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from fp_common.transforms import load_calibration, project_points

from fp_surface_guard.msg import Surface2D, SurfaceArray


class SurfaceBuilder(Node):
    def __init__(self):
        super().__init__('surface_builder')
        d = self.declare_parameter
        self.glass_ids = list(d('glass_ids', [8, 147]).value)
        self.water_ids = list(d('water_ids', [21, 60, 109]).value)
        self.stair_ids = list(d('stair_ids', [53, 59, 121]).value)
        self.wall_ids = list(d('wall_ids', [0, 5]).value)
        self.negative = d('negative_mode', True).value
        self.gap_px = int(d('gap_px', 20).value)
        self.min_area = int(d('min_area_px', 1500).value)
        self.pt_z_min = d('nonground_z_min', 0.10).value
        self.ground_z = d('ground_z_lidar', -0.525).value
        self.lidar_in_base = np.asarray(d('lidar_xyz_in_base', [0.25, 0.0, 0.525]).value,
                                        dtype=np.float64)
        self.strip_depth = d('strip_depth_m', 0.6).value
        self.seed_check = d('seed_check_m', 1.0).value
        self.max_range = d('max_range_m', 12.0).value
        self.hold_sec = d('hold_sec', 4.0).value
        self.roi_hold = d('roi_hold_sec', 30.0).value

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
        self.pub = self.create_publisher(SurfaceArray, '/fp_guard/surfaces', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/fp_guard/markers', 10)
        self.create_subscription(Image, '/fp/labels', self.cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/points_raw', self.pts_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/fp_sem/manual_roi', self.roi_cb, 10)
        self.raw_pts = None
        self.roi_line, self.roi_stamp = None, None
        self.last = {}            # cls -> (ring Nx3 base_link, stamp, conf, zmax)
        self.locked = {}          # cls -> (world_frame, world_ring Nx2, centroid)
        self.lock_radius = d('lock_radius_m', 0.7).value
        self.get_logger().info(
            'surface_builder up (vision-authority, lidar-seed geometry): '
            'glass=%s water=%s stair=%s negative=%s wall=%s'
            % (self.glass_ids, self.water_ids, self.stair_ids, self.negative, self.wall_ids))

    # ---------------- 输入 ----------------
    def pts_cb(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        off = {f.name: f.offset for f in msg.fields}
        p = np.stack([raw[:, off[a]:off[a] + 4].copy().view(np.float32).ravel()
                      for a in 'xyz'], 1)
        pb = p + self.lidar_in_base
        self.raw_pts = pb[pb[:, 2] > self.pt_z_min]

    def roi_cb(self, msg):
        pts = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        if len(pts) < 3:
            return
        us, vs = [p[0] for p in pts], [p[1] for p in pts]
        cols = np.arange(min(us), max(us), 2.0)
        self.roi_line = np.stack([cols, np.full_like(cols, max(vs) + 2.0)], 1)
        self.roi_stamp = self.get_clock().now().nanoseconds

    # ---------------- 几何 ----------------
    def ground_points(self, uv):
        """像素(脚线) -> base_link 地面点"""
        hom = np.stack([uv[:, 0], uv[:, 1], np.ones(len(uv))], 1)
        dv = hom @ self.K_inv.T
        denom = dv @ self.c3
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (self.ground_z + self.c3 @ self.t) / denom
        ok = (denom < -1e-9) & (s > 0.2) & (s < 40.0)
        p_cam = dv[ok] * s[ok][:, None]
        return (p_cam - self.t) @ self.R + self.lidar_in_base

    def project_pts_to_rows(self, pts):
        """base_link 点 -> 像素 (uv, depth)"""
        pl = pts[:, :3] - self.lidar_in_base
        return project_points(pl, self.K, self.R, self.t)

    def seeds_in_blob(self, blob):
        if self.raw_pts is None or len(self.raw_pts) == 0:
            return None
        uv, depth, _ = self.project_pts_to_rows(self.raw_pts)
        H, W = blob.shape
        ok = ((depth > 0.3) & (uv[:, 0] >= 0) & (uv[:, 0] < W) &
              (uv[:, 1] >= 0) & (uv[:, 1] < H))
        uvi = uv[ok].astype(int)
        inside = blob[uvi[:, 1], uvi[:, 0]]
        pts = self.raw_pts[ok][inside]
        return pts if len(pts) >= 3 else None

    def solid_occ_rows(self, W, H):
        if self.raw_pts is None:
            return {}
        uv, depth, _ = self.project_pts_to_rows(self.raw_pts)
        ok = (depth > 0.3) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & \
             (uv[:, 1] >= 0) & (uv[:, 1] < H)
        occ = {}
        for u, v in uv[ok].astype(int):
            for du in range(-3, 4):
                c2 = u + du
                if 0 <= c2 < W:
                    occ[c2] = max(occ.get(c2, -1), int(v))
        return occ

    def blobs(self, mask):
        k = np.ones((5, 5), np.uint8)
        m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        n, lmap, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        return [lmap == i for i in range(1, n)
                if stats[i, cv2.CC_STAT_AREA] >= self.min_area]

    def bottom_edge(self, blob):
        cols = np.where(blob.any(axis=0))[0]
        vs = np.array([np.where(blob[:, c])[0].max() for c in cols], np.float64)
        return np.stack([cols.astype(np.float64), vs + 2.0], 1)

    def negative_line(self, blob, occ, H):
        cand = []
        cols = np.where(blob.any(axis=0))[0]
        for c in cols:
            vb = int(np.where(blob[:, c])[0].max())
            vp = occ.get(int(c), -1)
            # 同 frame_line: 该列必须有实体回波(穿透面板打到后墙)才算玻璃;
            # 无回波列(超量程真墙/车体)一律排除
            if vp >= 0 and vb - vp >= self.gap_px and \
               H * 0.35 < vb < H * 0.85:
                cand.append((c, vb))
        if len(cand) < max(10, len(cols) * 0.3):
            return None
        return np.array(cand, dtype=np.float64)

    def frame_line(self, blob, occ, H):
        """玻璃有缺口列(无回波) + 框/柱自有回波列(回波紧贴blob底) -> 用地面立柱回波行定脚线,
        避免 wall blob 渗漏地板像素造成的深度偏差, 也避免近处托盘污染. 找不到则 None."""
        cols = np.where(blob.any(axis=0))[0]
        vb = np.array([np.where(blob[:, c])[0].max() for c in cols], np.float64)
        vp = np.array([occ.get(int(c), -1) for c in cols], np.float64)
        # 关键: 候选列必须有实体回波证据(vp>=0, 激光穿透面板打到后方墙面)。
        # 无回波(vr=-1)的列 = 超出激光量程的远处真墙/车体像素 -> 一律排除,
        # 否则负候选会把一切看不见的东西都报成玻璃(此前东墙假带困住机器人的根因)。
        gap_cols = ((vp >= 0) & (vb - vp >= self.gap_px) &
                    (vb > H * 0.35) & (vb < H * 0.85))
        if gap_cols.sum() < max(10, len(cols) * 0.3):
            return None
        lo, hi = int(cols[gap_cols].min()), int(cols[gap_cols].max())
        sel = (vp > H * 0.30) & (cols >= lo) & (cols <= hi)
        if sel.sum() < 4:
            return None
        # 取候选列内实体回波的"最深行簇"(上 10% 分位)：面板框/柱自有回波远深于
        # 穿透玻璃打到的后墙回波; 75% 分位会被后墙大簇淹没(此前把带锚到 11.8m 东墙)
        v_hi = float(np.percentile(vp[sel], 90))
        top = sel & (vp >= v_hi)
        v_cap = float(np.median(vb[gap_cols])) + 2.0
        v_foot = min(float(np.median(vp[top])) + 3.0, v_cap)
        us = np.linspace(lo, hi, 60)
        return np.stack([us, np.full_like(us, v_foot)], 1)

    # ---------------- 主流程 ----------------
    def cb(self, msg):
        lab = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        H, W = lab.shape
        found = {}
        blob_of = {}

        # 1) 玻璃直检（视觉权威）
        for blob in self.blobs(np.isin(lab, self.glass_ids)):
            self.add_vertical(found, blob, self.bottom_edge(blob), 0.9, msg.header.stamp)
        # 2) 负候选（激光补盲 -> 仅产生候选，深度仍由种子修正）
        if self.negative and self.raw_pts is not None and len(self.raw_pts) >= 10:
            occ = self.solid_occ_rows(W, H)
            for blob in self.blobs(np.isin(lab, self.wall_ids)):
                if 'glass' in found:
                    break
                line = self.frame_line(blob, occ, H)
                if line is None:
                    line = self.negative_line(blob, occ, H)
                if line is not None:
                    self.add_vertical(found, None, line, 0.85, msg.header.stamp)
        # 3) 手动 ROI 兜底
        if 'glass' not in found and self.roi_line is not None and \
           abs((msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec - self.roi_stamp)
               * 1e-9) < self.roi_hold:
            self.add_vertical(found, None, self.roi_line, 0.9, msg.header.stamp)
        # 4) 积水 / 楼梯口：贴地面掩码包络
        for cls, ids in (('water', self.water_ids), ('stair_down', self.stair_ids)):
            for blob in self.blobs(np.isin(lab, ids)):
                self.add_ground_patch(found, cls, blob, msg.header.stamp)

        if found:
            self.last = found
        self.publish(msg.header.stamp)

    def add_vertical(self, found, blob, uv, conf, stamp):
        pts = self.ground_points(uv)
        if pts is None or len(pts) < 2:
            return
        r = np.linalg.norm(pts[:, :2], axis=1)
        pts = pts[r < self.max_range]
        if len(pts) < 2:
            return
        # 种子修正(仅直检玻璃路径): blob 内回波中位距离与掩码底沿不符 -> 径向搬移
        if blob is not None:
            seeds = self.seeds_in_blob(blob)
            if seeds is not None:
                d_seed = float(np.median(np.linalg.norm(seeds[:, :2], axis=1)))
                d_foot = float(np.median(np.linalg.norm(pts[:, :2], axis=1)))
                if abs(d_seed - d_foot) > self.seed_check and d_foot > 0.3:
                    pts = pts * float(np.clip(d_seed / d_foot, 0.4, 2.5))
        found['glass'] = (self._strip(pts), stamp, conf, 1.5)

    def _strip(self, pts):
        p0, p1 = pts[0][:2], pts[-1][:2]
        n = np.array([-(p1 - p0)[1], (p1 - p0)[0]])
        n = n / max(np.linalg.norm(n), 1e-6)
        if np.dot(n, (p0 + p1) * 0.5) < 0:
            n = -n
        h = self.strip_depth * 0.5
        return [np.array([q[0], q[1], 0.0])
                for q in (p0 - n * h, p1 - n * h, p1 + n * h, p0 + n * h)]

    def add_ground_patch(self, found, cls, blob, stamp):
        vs = np.where(blob.any(axis=1))[0]
        if len(vs) == 0:
            return
        rows = []
        step = max(1, len(vs) // 12)
        for v in vs[::step]:
            cols = np.where(blob[v])[0]
            rows.extend([(cols.min(), v + 0.5), (cols.max(), v + 0.5)])
        pts = self.ground_points(np.array(rows, np.float64))
        if pts is None or len(pts) < 3:
            return
        pts = pts[np.linalg.norm(pts[:, :2], axis=1) < self.max_range]
        if len(pts) < 3:
            return
        hull = cv2.convexHull(pts[:, :2].astype(np.float32)).reshape(-1, 2)
        if len(hull) < 3 or len(hull) > 64:
            return
        conf = 0.7 if cls == 'water' else 0.6
        found[cls] = ([np.array([p[0], p[1], 0.0]) for p in hull], stamp, conf, 0.3)

    def to_world(self, ring):
        """base_link 环 -> 世界系(map, 退到 odom)。物体是静态的：锚定世界系后
        用滞回锁冻结几何，避免检测抖动让致命带跟着机器人漂移。"""
        t = None
        for f in ('map', 'odom'):
            try:
                t = self.buf.lookup_transform(f, 'base_link', rclpy.time.Time(),
                                              timeout=rclpy.duration.Duration(seconds=0.05))
                world = f
                break
            except Exception:
                continue
        if t is None:
            return 'base_link', np.asarray([p[:2] for p in ring])
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        Rb = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        tr = np.array([t.transform.translation.x, t.transform.translation.y,
                       t.transform.translation.z])
        pts = np.asarray([p[:3] for p in ring], np.float64) @ Rb.T + tr
        return world, pts[:, :2]

    def locked_ring(self, cls, ring):
        world, pts = self.to_world(ring)
        cen = pts.mean(axis=0)
        old = self.locked.get(cls)
        if old is not None and old[0] == world and np.linalg.norm(old[2] - cen) < self.lock_radius:
            return world, old[1]          # 滞回: 保持已锁定几何, 不跟随抖动
        self.locked[cls] = (world, pts, cen)
        return world, pts

    def publish(self, stamp):
        arr = SurfaceArray()
        arr.header = Header(stamp=stamp, frame_id='base_link')
        mk = MarkerArray()
        i = 0
        for cls, (ring, s, conf, zmax) in sorted(self.last.items()):
            dt = abs((stamp.sec * 1e9 + stamp.nanosec - (s.sec * 1e9 + s.nanosec)) * 1e-9)
            if dt > self.hold_sec:
                continue
            world, wpts = self.locked_ring(cls, ring)
            su = Surface2D()
            su.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=world)
            su.class_name = cls
            su.confidence = float(conf)
            su.polygon = Polygon(points=[Point32(x=float(p[0]), y=float(p[1]), z=0.0)
                                         for p in wpts])
            su.min_height = 0.0
            su.max_height = float(zmax)
            arr.surfaces.append(su)
            m = Marker()
            m.header = su.header
            m.ns = 'guard'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.1
            col = {'glass': (0.2, 0.6, 1.0), 'water': (0.1, 0.9, 0.9),
                   'stair_down': (1.0, 0.5, 0.0)}.get(cls, (1.0, 0.0, 1.0))
            m.color.r, m.color.g, m.color.b, m.color.a = col[0], col[1], col[2], 0.8
            m.header = su.header
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.05) for p in wpts]
            m.points.append(m.points[0])
            mk.markers.append(m)
            i += 1
        self.pub.publish(arr)
        self.pub_mk.publish(mk)


def main():
    rclpy.init()
    n = SurfaceBuilder()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
