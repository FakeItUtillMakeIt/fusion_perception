#!/usr/bin/env python3
"""障碍物检测（Phase 1: general_obstacle 几何分支）。

非地面点云 -> DBSCAN 聚类 -> 凸包+RDP 简化 -> ObstacleArray + MarkerArray。
含轻量跨帧 id 跟踪（最近邻匹配）。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        self.pub = self.create_publisher(ObstacleArray, '/fp_ob/obstacles', 10)
        self.pub_m = self.create_publisher(MarkerArray, '/fp_ob/markers', 10)
        self.create_subscription(PointCloud2, '/fp_ob/points_non_ground', self.cb, qos_profile_sensor_data)
        self.tracks = []
        self.next_id = 1
        self.get_logger().info('obstacle_detect up (general_obstacle geometric branch)')

    def cluster(self, p):
        tree = cKDTree(p[:, :2])
        pr = tree.query_pairs(self.eps, output_type='ndarray')
        if len(pr) == 0:
            return []
        a = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(p), len(p)))
        nc, lab = connected_components(a, directed=False)
        return [np.where(lab == i)[0] for i in range(nc)]

    def match_id(self, c):
        best, bd = None, self.match_dist
        for tr in self.tracks:
            dd = np.hypot(tr['c'][0] - c[0], tr['c'][1] - c[1])
            if dd < bd:
                best, bd = tr, dd
        now = self.get_clock().now().nanoseconds * 1e-9
        if best is None:
            ob = Obstacle()
            ob.id = self.next_id
            self.next_id += 1
            self.tracks.append({'id': ob.id, 'c': c, 't': now})
            return ob.id
        best['c'], best['t'] = c, now
        return best['id']

    def cb(self, msg):
        p = parse_xyzt(msg)
        if len(p) < self.min_pts:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.tracks = [t for t in self.tracks if now - t['t'] < self.stale]

        arr = ObstacleArray()
        arr.header = msg.header
        markers = MarkerArray()
        mid = 0
        for idx in self.cluster(p):
            cpts = p[idx]
            if len(cpts) < self.min_pts:
                continue
            cen = cpts.mean(axis=0)
            if np.hypot(cen[0], cen[1]) > self.max_dist:
                continue
            try:
                hull = ConvexHull(cpts[:, :2])
            except Exception:
                continue
            hp = cpts[hull.vertices, :2]
            if hp.max(axis=0)[0] - hp.min(axis=0)[0] < self.min_size and \
               hp.max(axis=0)[1] - hp.min(axis=0)[1] < self.min_size:
                continue
            ap = cv2.approxPolyDP(hp.reshape(-1, 1, 2).astype(np.float32), self.rdp, True).reshape(-1, 2)
            if len(ap) < 3:
                continue

            ob = Obstacle()
            ob.header = msg.header
            ob.class_name = 'general_obstacle'
            ob.source = 'geometric'
            ob.confidence = float(min(1.0, len(cpts) / 20.0))
            ob.id = self.match_id(cen[:2])
            ob.min_height = float(cpts[:, 2].min())
            ob.max_height = float(cpts[:, 2].max())
            ob.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in ap]
            arr.obstacles.append(ob)

            mk = Marker()
            mk.header = msg.header
            mk.ns = 'obstacles'
            mk.id = mid
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.04
            mk.color.g = 0.9
            mk.color.a = 1.0
            mk.points = [Point(x=float(q.x), y=float(q.y), z=0.0)
                         for q in list(ob.polygon.points) + [ob.polygon.points[0]]]
            markers.markers.append(mk)
            tx = Marker()
            tx.header = msg.header
            tx.ns = 'labels'
            tx.id = mid + 1000
            tx.type = Marker.TEXT_VIEW_FACING
            tx.action = Marker.ADD
            tx.pose.position.x = float(cen[0])
            tx.pose.position.y = float(cen[1])
            tx.pose.position.z = float(cen[2]) + 0.3
            tx.scale.z = 0.25
            tx.color.r, tx.color.g, tx.color.b, tx.color.a = 1.0, 1.0, 0.0, 1.0
            tx.text = f'#{ob.id} {ob.class_name}'
            markers.markers.append(tx)
            mid += 1

        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.ns = 'stale'
        markers.markers.insert(0, del_m)
        self.pub.publish(arr)
        self.pub_m.publish(markers)


def main():
    rclpy.init()
    n = DetectNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
