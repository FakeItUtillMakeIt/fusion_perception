#!/usr/bin/env python3
"""障碍物检测（Phase 1: general_obstacle 几何分支）。

非地面点云 -> DBSCAN 聚类 -> 凸包+RDP 简化 -> ObstacleArray + MarkerArray。
含轻量跨帧 id 跟踪（最近邻匹配）。
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from scipy.spatial import cKDTree, ConvexHull
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import cv2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Polygon, Point32, Point
from visualization_msgs.msg import Marker, MarkerArray
from fp_ob_msgs.msg import Obstacle, ObstacleArray


def parse_xyzt(msg):
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    return np.stack([raw[:, off[a]:off[a] + 4].copy().view(np.float32).ravel() for a in 'xyz'], axis=1)


class DetectNode(Node):
    def __init__(self):
        super().__init__('obstacle_detect')
        d = self.declare_parameter
        self.eps = d('cluster_eps', 0.15).value
        self.min_pts = d('min_points', 4).value
        self.max_dist = d('max_centroid_dist', 18.0).value
        self.rdp = d('polygon_simplify_eps', 0.05).value
        self.match_dist = d('track_match_dist', 0.6).value
        self.stale = d('track_stale', 2.0).value
        self.min_size = d('min_polygon_size', 0.15).value
        self.max_extent = d('max_obb_extent', 10.0).value

        self.pub = self.create_publisher(ObstacleArray, '/fp_ob/obstacles', 10)
        self.pub_m = self.create_publisher(MarkerArray, '/fp_ob/markers', 10)
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.world_frame = self.declare_parameter('world_frame', 'odom').value
        self.create_subscription(PointCloud2, '/fp_ob/points_non_ground', self.cb, qos_profile_sensor_data)
        self.tracks = []
        self.next_id = 1
        self.rob = np.zeros(2)
        self.max_miss = self.declare_parameter('max_miss_frames', 5).value
        self.live_marker_ids = set()
        self.get_logger().info('obstacle_detect up (general_obstacle geometric branch, coasting on)')

    def cluster(self, p):
        tree = cKDTree(p[:, :2])
        pr = tree.query_pairs(self.eps, output_type='ndarray')
        if len(pr) == 0:
            return []
        a = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(p), len(p)))
        nc, lab = connected_components(a, directed=False)
        return [np.where(lab == i)[0] for i in range(nc)]

    def match_track(self, cen, ob, used):
        xs = [p.x for p in ob.polygon.points]
        ys = [p.y for p in ob.polygon.points]
        new_bb = (min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5)
        best, bd = None, self.match_dist
        for tr in self.tracks:
            if tr['id'] in used:
                continue
            tbb = tr.get('bb')
            dd = np.hypot(tr['ob_c'][0] - cen[0], tr['ob_c'][1] - cen[1])
            overlap = tbb is not None and not (
                new_bb[0] > tbb[2] or new_bb[2] < tbb[0] or
                new_bb[1] > tbb[3] or new_bb[3] < tbb[1])
            score = dd if not overlap else min(dd, 1.5)
            if score < bd:
                best, bd = tr, score
        if best is None:
            tr = {'id': self.next_id, 'ob_c': cen.copy(), 'miss': 0, 'bb': new_bb}
            self.next_id += 1
            self.tracks.append(tr)
            best = tr
        else:
            used.add(best['id'])
            best['miss'] = 0
            a = 0.5
            best['ob_c'] = a * best['ob_c'] + (1 - a) * cen
            best['bb'] = new_bb
        best['ob'] = ob
        best['ob'].id = best['id']
        best['ob'].confidence = float(min(1.0, ob.confidence + 0.1))
        return best

    def cb(self, msg):
        p = parse_xyzt(msg)
        if not hasattr(self, '_dbg2'): self._dbg2 = 0
        self._dbg2 += 1
        if self._dbg2 <= 2:
            self.get_logger().info('cb in: n=%d frame=%s wf=%s' % (len(p), msg.header.frame_id, self.world_frame))
        try:
            tfm = self.buf.lookup_transform(self.world_frame, msg.header.frame_id, msg.header.stamp)
        except Exception:
            try:
                tfm = self.buf.lookup_transform(self.world_frame, msg.header.frame_id,
                                                rclpy.time.Time())
            except Exception as e:
                if not hasattr(self, '_dbg'): self._dbg = 0
                self._dbg += 1
                if self._dbg in (3, 50, 200):
                    self.get_logger().warn('tf fail #%d: %s | frames=%s' % (self._dbg, str(type(e))[:40], self.buf.all_frames_as_string().replace(chr(10),'->')[:200]))
                return
        q = tfm.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        t = np.array([tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z])
        self.rob = t[:2]
        p = p @ R.T + t
        msg.header.frame_id = self.world_frame
        now = self.get_clock().now().nanoseconds * 1e-9
        used = set()
        if len(p) >= self.min_pts:
            for idx in self.cluster(p):
                cpts = p[idx]
                if len(cpts) < self.min_pts:
                    continue
                cen = cpts.mean(axis=0)
                if np.hypot(cen[0] - self.rob[0], cen[1] - self.rob[1]) > self.max_dist:
                    continue
                try:
                    hull = ConvexHull(cpts[:, :2])
                except Exception:
                    continue
                hp = cpts[hull.vertices, :2]
                if hp.max(axis=0)[0] - hp.min(axis=0)[0] < self.min_size and \
                   hp.max(axis=0)[1] - hp.min(axis=0)[1] < self.min_size:
                    continue
                rect = cv2.minAreaRect(hp.astype(np.float32))
                (cx, cy), (bw, bl), bang = rect
                if bw < self.min_size or bl < self.min_size:
                    continue
                if bw > self.max_extent or bl > self.max_extent:
                    continue
                box = cv2.boxPoints(rect).astype(np.float64)
                ob = Obstacle()
                ob.header = msg.header
                ob.class_name = 'general_obstacle'
                ob.source = 'geometric'
                ob.confidence = float(min(0.9, len(cpts) / 20.0))
                ob.min_height = float(cpts[:, 2].min())
                ob.max_height = float(cpts[:, 2].max())
                ob.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in box]
                tr = self.match_track(cen[:2], ob, used)
                tr['obb'] = (float(cx), float(cy), float(bw), float(bl), float(bang),
                             ob.min_height, ob.max_height)

        for tr in self.tracks:
            if tr['id'] not in used:
                tr['miss'] += 1
                if tr['miss'] > self.max_miss:
                    tr['ob'] = None
                elif tr['ob'] is not None:
                    tr['ob'].confidence = max(0.1, tr['ob'].confidence * 0.85)
        self.tracks = [t for t in self.tracks if t['ob'] is not None]

        arr = ObstacleArray()
        arr.header = msg.header
        markers = MarkerArray()
        cur_ids = set()
        for tr in self.tracks:
            ob = tr['ob']
            arr.obstacles.append(ob)
            mk = Marker()
            mk.header = msg.header
            mk.ns = 'obstacles'
            mk.id = tr['id'] * 2
            mk.type = Marker.LINE_LIST
            mk.action = Marker.ADD
            mk.scale.x = 0.03
            mk.color.g = 0.9
            mk.color.a = 1.0 if tr['miss'] == 0 else 0.4
            cx, cy, bw, bl, bang, z0, z1 = tr.get('obb', (0, 0, 0.1, 0.1, 0, 0, 0.5))
            co, so = np.cos(np.radians(bang)), np.sin(np.radians(bang))
            hw, hl = bw / 2, bl / 2
            cs = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
            plan = [(cx + x * co - y * so, cy + x * so + y * co) for x, y in cs]
            # 顶点顺序：0-3 底环，4-7 顶环（与棱索引表配对）
            verts = [Point(x=px, y=py, z=z0) for px, py in plan] + \
                    [Point(x=px, y=py, z=z1) for px, py in plan]
            edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 6), (3, 7)]
            mk.points = [verts[a] for e in edges for a in e]
            markers.markers.append(mk)
            # 实心长方体：中心/朝向/尺寸全部来自 OBB
            cube = Marker()
            cube.header = msg.header
            cube.ns = 'cubes'
            cube.id = tr['id'] * 2
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            yaw = math.atan2(plan[1][1] - plan[0][1], plan[1][0] - plan[0][0])
            L = math.hypot(plan[1][0] - plan[0][0], plan[1][1] - plan[0][1])
            W = math.hypot(plan[2][0] - plan[1][0], plan[2][1] - plan[1][1])
            H = max(0.1, z1 - z0)
            cube.pose.position.x = sum(p[0] for p in plan) / 4
            cube.pose.position.y = sum(p[1] for p in plan) / 4
            cube.pose.position.z = (z0 + z1) / 2
            cube.pose.orientation.z = math.sin(yaw / 2)
            cube.pose.orientation.w = math.cos(yaw / 2)
            cube.scale.x, cube.scale.y, cube.scale.z = L, W, H
            cube.color.g = 0.7
            cube.color.a = 0.35 if tr['miss'] == 0 else 0.15
            markers.markers.append(cube)
            tx = Marker()
            tx.header = msg.header
            tx.ns = 'obstacles'
            tx.id = tr['id'] * 2 + 1
            tx.type = Marker.TEXT_VIEW_FACING
            tx.action = Marker.ADD
            cen = tr['ob_c']
            tx.pose.position.x = float(cen[0])
            tx.pose.position.y = float(cen[1])
            tx.pose.position.z = ob.max_height + 0.3
            tx.scale.z = 0.25
            tx.color.r, tx.color.g, tx.color.b, tx.color.a = 1.0, 1.0, 0.0, 1.0
            tx.text = f'#{ob.id} {ob.class_name}'
            markers.markers.append(tx)
            cur_ids.update((tr['id'] * 2, tr['id'] * 2 + 1))
        for gone in self.live_marker_ids - cur_ids:
            for ns in ('obstacles', 'cubes'):
                dm = Marker()
                dm.header = msg.header
                dm.ns = ns
                dm.id = gone
                dm.action = Marker.DELETE
                markers.markers.append(dm)
        self.live_marker_ids = cur_ids
        self.pub.publish(arr)
        self.pub_m.publish(markers)


def main():
    rclpy.init()
    n = DetectNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
