#!/usr/bin/env python3
"""GlassGuard 完整流水线（论文四阶段的本仓库实现，独立包，不改旧代码）。

① 掩码提案源: /fp/labels 玻璃类直检 + /fp_sem/manual_roi(真机换微调实例模型, 接口不变)
② 柱状结构建候选: 激光体素"垂直柱"(多层占据) = 窗框/立柱等稳定几何证据;
   由柱界定有界竖直平面(位置+朝向+范围), 不拟合玻璃自身稀疏回波, 不用单目深度
③ 无深度投影验证: 平面四角投回图像 vs 掩码 IoU; 掩码内部在平面深度带内必须"空腔"
   (实心墙内部全是回波 -> 拒绝); 法向朝向检查
④ 全局假设管理: map 系假设表, 种子支持度 EMA; 同类合并(线参数距离/角度门),
   多视重投影: 投影落到掩码外比例升高 -> 衰减直至删除; floor eviction:
   条带落在连续地面回波上且无柱支撑 -> 立即驱逐(防"地面被标成玻璃/墙"围死机器人)

输出: /fp_guard/surfaces (fp_surface_guard.msg.SurfaceArray, map 系)
      /fp_glassguard/markers (假设可视化: 轮廓+支持度)
"""
import math
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
import tf2_geometry_msgs  # noqa: F401
from visualization_msgs.msg import Marker, MarkerArray
from fp_common.transforms import load_calibration, project_points
from fp_surface_guard.msg import Surface2D, SurfaceArray

TWO_PI = 2.0 * math.pi


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Hypothesis:
    _next_id = 1

    def __init__(self, p0, p1, zmax, support, conf, frame='map'):
        self.id = Hypothesis._next_id
        Hypothesis._next_id += 1
        self.frame = frame
        self.p0 = np.asarray(p0, np.float64)   # 底边端点 (世界系 xy)
        self.p1 = np.asarray(p1, np.float64)
        self.zmax = float(zmax)
        self.support = float(support)          # 柱证据支持度
        self.conf = float(conf)
        self.hits = 1
        self.misses = 0
        self.stamp = None

    def line_params(self):
        d = self.p1 - self.p0
        L = max(np.linalg.norm(d), 1e-6)
        return (self.p0 + self.p1) * 0.5, d / L, L

    def strip_poly(self, half_w=0.25):
        c, u, _ = self.line_params()
        n = np.array([-u[1], u[0]])
        a = self.p0 - n * half_w
        b = self.p1 - n * half_w
        e = self.p1 + n * half_w
        f = self.p0 + n * half_w
        return np.stack([a, b, e, f])


class GlassGuardNode(Node):
    def __init__(self):
        super().__init__('glassguard')
        d = self.declare_parameter
        self.glass_ids = list(d('glass_ids', [8, 147, 130]).value)
        self.roi_hold = d('roi_hold_sec', 30.0).value
        self.voxel = d('voxel_m', 0.1).value
        self.min_layers = int(d('min_layers', 3).value)
        self.band_m = d('depth_band_m', 0.5).value
        self.min_support = int(d('min_support_cols', 3).value)
        self.min_iou = d('min_iou', 0.70).value
        self.max_interior = d('max_interior_frac', 0.75).value
        self.merge_dist = d('merge_dist_m', 0.6).value
        self.merge_ang = math.radians(d('merge_angle_deg', 20.0).value)
        self.keep_iou = d('keep_iou', 0.12).value
        self.max_miss = int(d('max_miss_frames', 25).value)
        self.decay = d('support_decay', 0.85).value
        self.min_pub_support = float(d('min_pub_support', 2.0).value)
        self.hyp_ttl = d('hyp_ttl_s', 15.0).value
        self.zmax = d('panel_height_m', 1.8).value
        self.max_range = d('max_range_m', 12.0).value
        self.floor_evict_frac = d('floor_evict_frac', 0.6).value
        self.foot_gate = d('foot_gate_m', 2.5).value
        self.ground_z = d('ground_z_lidar', -0.525).value
        self.lidar_in_base = np.asarray(
            d('lidar_xyz_in_base', [0.25, 0.0, 0.525]).value, np.float64)
        self.use_negative = d('negative_mode', False).value
        self.wall_ids = list(d('wall_ids', [0, 5]).value)

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
        self.tf_listener = tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(SurfaceArray, '/fp_guard/surfaces', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/fp_glassguard/markers', 10)
        self.create_subscription(Image, '/fp/labels', self.cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/points_raw', self.pts_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/fp_sem/manual_roi', self.roi_cb, 10)

        self.raw_base = None      # base_link 全高度原始点
        self.roi_mask = None      # 最近 ROI 的像素多边形
        self.roi_stamp = None
        self.hyps = []            # 阶段④ 假设表 (世界系)
        self.world_frame = 'map'
        self.get_logger().info(
            'glassguard up: voxel=%.2f min_support=%d iou>=%.2f interior<=%.2f '
            'glass_ids=%s negative=%s' % (self.voxel, self.min_support, self.min_iou,
                                          self.max_interior, self.glass_ids,
                                          self.use_negative))

    # ---------------- 输入 ----------------
    def pts_cb(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        off = {f.name: f.offset for f in msg.fields}
        p = np.stack([raw[:, off[a]:off[a] + 4].copy().view(np.float32).ravel()
                      for a in 'xyz'], 1)
        self.raw_base = p + self.lidar_in_base

    def roi_cb(self, msg):
        pts = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        if len(pts) >= 3:
            self.roi_mask = np.asarray(pts, np.float64)
            self.roi_stamp = self.get_clock().now().nanoseconds
            self.get_logger().info('roi accepted: %d pts' % len(pts))

    # ---------------- 几何工具 ----------------
    def ground_points(self, uv):
        hom = np.stack([uv[:, 0], uv[:, 1], np.ones(len(uv))], 1)
        dv = hom @ self.K_inv.T
        denom = dv @ self.c3
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (self.ground_z + self.c3 @ self.t) / denom
        ok = (denom < -1e-9) & (s > 0.2) & (s < 40.0)
        p_cam = dv[ok] * s[ok][:, None]
        return (p_cam - self.t) @ self.R + self.lidar_in_base

    def project_base(self, pts):
        """base_link 点 -> 像素 uv, 相机深度"""
        if len(pts) == 0:
            return np.zeros((0, 2)), np.zeros(0)
        return project_points(np.asarray(pts)[:, :3] - self.lidar_in_base,
                              self.K, self.R, self.t)[:2]

    def base_to_map(self, pts):
        """base -> map, 定位断链/滞后时退到 odom(假设表允许混合帧? 否——
        退化时整帧用 odom, 由 frame 字段随假设记录)"""
        import rclpy.duration
        for frame in ('map', 'odom'):
            try:
                tfm = self.buf.lookup_transform(frame, 'base_link', rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=0.1))
                break
            except Exception:
                tfm = None
        if tfm is None:
            raise RuntimeError('tf map/odom -> base unavailable')
        self.world_frame = frame
        q = tfm.transform.rotation
        Rm = quat_to_R(q.x, q.y, q.z, q.w)
        tr = np.array([tfm.transform.translation.x, tfm.transform.translation.y,
                       tfm.transform.translation.z])
        return np.asarray(pts)[:, :3] @ Rm.T + tr

    # ---------------- 阶段② 柱状结构 ----------------
    def vertical_columns(self):
        """体素垂直柱: (ix,iy) -> 3D 中心点, 要求占据 >= min_layers 个高度层"""
        if self.raw_base is None or len(self.raw_base) < 20:
            return np.zeros((0, 3))
        p = self.raw_base
        k = int(1.0 / self.voxel)
        key = (p[:, 0] * k).astype(int) * 100000 + (p[:, 1] * k).astype(int)
        zlay = (p[:, 2] * (1.0 / 0.05)).astype(int)
        out = []
        order = np.argsort(key)
        key_s, z_s = key[order], zlay[order]
        idx_s = order
        bounds = np.flatnonzero(np.diff(key_s)) + 1
        groups = np.split(np.arange(len(key_s)), bounds + 1)
        for g in groups:
            if len(g) == 0:
                continue
            if len(np.unique(z_s[g])) >= self.min_layers:
                i0 = idx_s[g[0]]
                cx = math.floor(p[i0, 0] / self.voxel) * self.voxel + self.voxel * 0.5
                cy = math.floor(p[i0, 1] / self.voxel) * self.voxel + self.voxel * 0.5
                out.append((cx, cy, float(p[idx_s[g], 2].mean())))
        return np.asarray(out) if out else np.zeros((0, 3))

    def fit_vertical_plane(self, cols, blob):
        """柱点(深度带内) -> 有界竖直平面: PCA 线方向 + 法向 + 端点范围"""
        if len(cols) < self.min_support:
            return None
        H, W = blob.shape
        dil = cv2.dilate(blob.astype(np.uint8), np.ones((25, 25), np.uint8)).astype(bool)
        uv, depth = self.project_base(cols)
        ok = (depth > 0.3) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & \
             (uv[:, 1] >= 0) & (uv[:, 1] < H)
        inside = ok.copy()
        ui = uv[ok].astype(int)
        inside[ok] = dil[ui[:, 1], ui[:, 0]]
        sel = np.flatnonzero(inside)
        if len(sel) < self.min_support:
            return None
        # 关键锚定: 取"最近深度簇"(10%分位 +1.2m 内) —— 面板永远在背景(后墙)之前,
        # 否则 PCA 会被穿透可见的后墙大簇劫持(拟合到 11.8m 东墙的根因)
        pts_all = cols[sel]
        dep = np.linalg.norm(pts_all[:, :2], axis=1)
        d10 = float(np.percentile(dep, 10))
        near = dep < d10 + 0.7
        pts = pts_all[near]
        if len(pts) < self.min_support:
            return None
        xy = pts[:, :2]
        mean = xy.mean(0)
        cov = np.cov((xy - mean).T)
        w, V = np.linalg.eigh(cov)
        u = V[:, -1]                       # 主方向 = 平面走向
        nrm = np.array([-u[1], u[0]])
        if np.dot(nrm, mean) < 0:
            nrm = -nrm                     # 法向远离相机(机器人原点) => 面朝相机
        t = (xy - mean) @ u
        tmin, tmax = t.min(), t.max()
        if tmax - tmin < 0.5:
            return None                    # 端点重合, 无界范围
        p0 = mean + u * tmin
        p1 = mean + u * tmax
        return p0, p1, len(sel), pts

    # ---------------- 阶段③ 无深度投影验证 ----------------
    def verify(self, p0, p1, blob, pts3):
        H, W = blob.shape
        c = (p0 + p1) * 0.5
        u = p1 - p0
        u = u / max(np.linalg.norm(u), 1e-6)
        nrm = np.array([-u[1], u[0]])
        if np.dot(nrm, c) < 0:
            nrm = -nrm
        corners_b = np.stack([
            [p0[0], p0[1], 0.05], [p1[0], p1[1], 0.05],
            [p1[0], p1[1], self.zmax], [p0[0], p0[1], self.zmax]])
        uv, _ = self.project_base(corners_b)
        if not np.isfinite(uv).all() or (uv[:, 0] < -W).any() or (uv[:, 0] > 2 * W).any():
            self._dbg = (p0, p1, uv.tolist())
            return 0.0
        quad = uv.astype(np.int32)
        pm = np.zeros((H, W), np.uint8)
        cv2.fillConvexPoly(pm, quad, 1)
        inter = float((pm & blob).sum())
        # 一致性度量 = 平面投影落在掩码内的比例(containment), 非 IoU:
        # 平面只覆盖柱支撑部分、掩码更大时 IoU 必然低, 但朝向一致性依然成立
        iou = inter / max(float(pm.sum()), 1.0)
        if iou < self.min_iou:
            self._dbg = (p0, p1, uv.tolist(), int(pm.sum()), int(blob.sum()))
        # 内部空腔: 平面深度带内的"高柱"(均高>0.5m)应只在边界附近(竖框/立柱),
        # 实心墙内部充满高柱 -> 拒绝。底轨(均高~0.1m 沿整条线)是面板合法构件,
        # 不计入实心证据, 否则真玻璃会被误拒。
        along = (pts3[:, :2] - c) @ u
        tall = pts3[:, 2] > 0.5
        near_band = (np.abs((pts3[:, :2] - c) @ nrm) < self.band_m) & tall
        span = max(np.linalg.norm(p1 - p0) * 0.5, 1e-3)
        # 连续性判据(替代占比): 实心墙沿线连续无隙(覆盖率~1.0);
        # 带中梃玻璃窗 = 周期竖梃 + 大段空腔(覆盖率~0.3-0.5)
        sel_along = along[near_band & (np.abs(along) < span)]
        n_s = max(int(2 * span / 0.3), 4)
        covered = 0
        for i in range(n_s):
            a = -span + (i + 0.5) * 2 * span / n_s
            if len(sel_along) and float(np.abs(sel_along - a).min()) < 0.12:
                covered += 1
        coverage = covered / n_s
        if coverage > self.max_interior:
            # 密网格窗与实心墙在沿线连续性上不可分 -> 用掩码底沿深度一致性旁路:
            # 真玻璃平面距离 ~= 掩码底沿地面投影距离; 假带(后墙平面)会差出米级
            vs = np.where(blob.any(axis=1))[0]
            foot_ok = False
            if len(vs):
                bot = vs[-1]
                cols_b = np.where(blob[bot])[0]
                uv_f = np.stack([cols_b[::max(1, len(cols_b)//20)],
                                 np.full(len(cols_b[::max(1, len(cols_b)//20)]),
                                         float(bot))], 1)
                fp = self.ground_points(uv_f)
                if len(fp):
                    d_foot = float(np.median(np.linalg.norm(fp[:, :2], axis=1)))
                    d_plane = float(np.linalg.norm(c))
                    foot_ok = abs(d_plane - d_foot) < getattr(self, 'foot_gate', 2.5)
            self._cov = coverage
            if not foot_ok:
                return 0.0                 # 沿线实心且深度与底沿矛盾 => 是墙
        return iou

    # ---------------- floor eviction (阶段④ 之一) ----------------
    def floor_contradiction(self, hyp):
        """条带上无柱支撑且被连续地面回波占据 => 是地面不是面板"""
        if self.raw_base is None:
            return False
        poly = hyp.strip_poly(0.30)
        c, u, L = hyp.line_params()
        cols_map = None
        try:
            cols_map = self.base_to_map(self.vertical_columns())
        except Exception:
            return False
        if len(cols_map):
            dcol = np.abs((cols_map[:, :2] - c) @ np.array([-u[1], u[0]]))
            acol = (cols_map[:, :2] - c) @ u
            on = (dcol < 0.4) & (acol > -0.5 * L - 0.4) & (acol < 0.5 * L + 0.4)
            if on.sum() >= 2:
                return False               # 条带上有竖直结构证据, 不驱逐
        fl = self.raw_base[(self.raw_base[:, 2] > -0.05) &
                           (self.raw_base[:, 2] < 0.15)]
        if len(fl) == 0:
            return False
        try:
            flm = self.base_to_map(fl)[:, :2]
        except Exception:
            return False
        uperp = np.array([-u[1], u[0]])
        dperp = np.abs((flm - c) @ uperp)
        apar = (flm - c) @ u                      # 相对质心: [-L/2, L/2]
        onstrip = (dperp < 0.25) & (apar > -0.5 * L) & (apar < 0.5 * L)
        samples = max(int(L / 0.3), 4)
        hit = 0
        for i in range(samples):
            s = (i + 0.5) * L / samples - 0.5 * L
            near = onstrip & (np.abs(apar - s) < 0.35)
            hit += int(near.any())
        return (hit / samples) > self.floor_evict_frac

    # ---------------- 主流程 ----------------
    def blobs(self, lab):
        out = []
        gm = np.isin(lab, self.glass_ids).astype(np.uint8)
        out += self._mask_blobs(gm)
        if self.use_negative and self.raw_base is not None:
            wm = np.isin(lab, self.wall_ids).astype(np.uint8)
            out += self._mask_blobs(wm)
        now = self.get_clock().now().nanoseconds
        if self.roi_mask is not None and abs((now - self.roi_stamp) * 1e-9) < self.roi_hold:
            H, W = lab.shape
            rm = np.zeros((H, W), np.uint8)
            cv2.fillPoly(rm, [self.roi_mask.astype(np.int32)], 1)
            out.append(rm.astype(bool))
        return out

    def _mask_blobs(self, mask):
        k = np.ones((5, 5), np.uint8)
        m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        n, lmap, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        return [lmap == i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] > 1200]

    def cb(self, msg):
        lab = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        cols = self.vertical_columns()
        seen = set()
        self._stat = getattr(self, '_stat', 0) + 1
        if self._stat % 30 == 0:
            blobs = self.blobs(lab)
            self.get_logger().info(
                'stats: cols=%d blobs=%d hyps=%d' % (len(cols), len(blobs), len(self.hyps)))
        if len(cols) >= self.min_support:
            try:
                cols_base = cols
                trace = []
                for blob in self.blobs(lab):
                    px = int(blob.sum())
                    fit = self.fit_vertical_plane(cols_base, blob)
                    if fit is None:
                        trace.append('%d:fitNone' % px)
                        continue
                    p0b, p1b, nsup, pts = fit
                    iou = self.verify(p0b, p1b, blob, pts)
                    if iou < self.min_iou:
                        trace.append('%d:rej(iou%.2f sup%d cov%.2f)'
                                     % (px, iou, nsup, getattr(self, '_cov', -1.0)))
                        continue
                    try:
                        p0b3 = np.array([p0b[0], p0b[1], 0.0])[None, :]
                        p1b3 = np.array([p1b[0], p1b[1], 0.0])[None, :]
                        p0m = self.base_to_map(p0b3)[0, :2]
                        p1m = self.base_to_map(p1b3)[0, :2]
                    except Exception as te:
                        trace.append('%d:tfFail(%r)' % (px, te))
                        continue
                    hid = self._merge_or_add(p0m, p1m, nsup, msg.header.stamp)
                    if hid is not None:
                        seen.add(hid)
                    trace.append('%d:OK(h%d sup%d)' % (px, hid or -1, nsup))
                if self._stat % 15 == 0:
                    self.get_logger().info('trace: ' + ' | '.join(trace))
            except Exception as e:
                self.get_logger().warn('pipeline: %r' % e)
        self._decay_unseen(seen, msg.header.stamp)
        self.publish(msg.header.stamp)

    # ---------------- 阶段④ 假设管理 ----------------
    def _merge_or_add(self, p0, p1, support, stamp):
        c = (p0 + p1) * 0.5
        d = p1 - p0
        Ln = max(np.linalg.norm(d), 1e-6)
        uh = d / Ln
        for h in self.hyps:
            if h.frame != getattr(self, 'world_frame', 'map'):
                continue
            hc, hu, hL = h.line_params()
            ang = math.acos(min(1.0, abs(float(np.dot(uh, hu)))))
            dist = abs(float(np.cross(np.array([hu[0], hu[1], 0.0]),
                                      np.array([c[0] - hc[0], c[1] - hc[1], 0.0]))[2]))
            along = float(np.dot(c - hc, hu))
            if ang < self.merge_ang and dist < self.merge_dist and \
               -0.5 < along < hL + 0.5:
                # 合并: 支持度加权保留更强几何, EMA 更新
                w = h.support / max(h.support + support, 1e-6)
                h.p0 = h.p0 * w + p0 * (1 - w)
                h.p1 = h.p1 * w + p1 * (1 - w)
                h.support = 0.7 * h.support + 0.3 * support
                h.conf = min(0.95, 0.45 + 0.08 * h.support)
                h.misses = 0
                h.stamp = stamp
                return h.id
        h = Hypothesis(p0, p1, self.zmax, support, min(0.95, 0.45 + 0.08 * support),
                       getattr(self, 'world_frame', 'map'))
        h.stamp = stamp
        self.hyps.append(h)
        self.get_logger().info('new hypothesis #%d L=%.1fm support=%d' % (h.id, Ln, support))
        return h.id

    def _decay_unseen(self, seen, stamp):
        now = self.get_clock().now().nanoseconds * 1e-9
        keep = []
        for h in self.hyps:
            if h.id in seen:
                keep.append(h)
                continue
            age = now - (h.stamp.sec + h.stamp.nanosec * 1e-9) if h.stamp else 0
            h.misses += 1
            h.support *= self.decay
            if h.misses > self.max_miss or age > self.hyp_ttl:
                self.get_logger().info(
                    'drop hypothesis #%d (misses=%d support=%.1f)'
                    % (h.id, h.misses, h.support))
                continue
            if self.floor_contradiction(h):
                self.get_logger().warn(
                    'FLOOR-EVICT #%d: 条带与连续地面回波矛盾, 立即删除' % h.id)
                continue
            keep.append(h)
        self.hyps = keep

    def publish(self, stamp):
        arr = SurfaceArray()
        arr.header = Header(stamp=stamp, frame_id=self.world_frame)
        mk = MarkerArray()
        for h in self.hyps:
            if h.support < self.min_pub_support:
                continue
            su = Surface2D()
            su.header = Header(stamp=stamp, frame_id=h.frame)
            su.class_name = 'glass'
            su.confidence = float(h.conf)
            poly = h.strip_poly()
            su.polygon = Polygon(points=[Point32(x=float(p[0]), y=float(p[1]), z=0.0)
                                         for p in poly])
            su.min_height = 0.0
            su.max_height = float(h.zmax)
            arr.surfaces.append(su)
            m = Marker()
            m.header = su.header
            m.ns = 'hyp'
            m.id = h.id
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 0.9
            q = poly[[0, 1, 1, 2, 2, 3, 3, 0]]
            for z in (0.0, 0.9, 1.8):
                pts = q.copy()
                pts = np.stack([pts[:, 0], pts[:, 1], np.full(len(pts), z)], 1)
                m.points.extend([Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                                 for p in pts])
            mk.markers.append(m)
            t = Marker()
            t.header = su.header
            t.ns = 'hyp'
            t.id = 100 + h.id
            t.type = Marker.TEXT_VIEW_FACING
            t.pose.position.x = float(np.mean(poly[:, 0]))
            t.pose.position.y = float(np.mean(poly[:, 1]))
            t.pose.position.z = 2.1
            t.scale.z = 0.22
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = 'glass#%d s=%.0f c=%.2f' % (h.id, h.support, h.conf)
            mk.markers.append(t)
        self.pub.publish(arr)
        self.pub_mk.publish(mk)


def main():
    rclpy.init()
    n = GlassGuardNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
