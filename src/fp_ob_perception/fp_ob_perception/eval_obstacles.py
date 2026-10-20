#!/usr/bin/env python3
"""Phase 1 评估：/fp_ob/obstacles vs factory1.world 真值区。

真值区从 world 文件几何硬编码（静态场景，真值免费）。
指标：分区召回、中心误差、虚警数（结构性墙体不计虚警）。
用法：python3 eval_obstacles.py [--duration 15]
"""
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from fp_ob_msgs.msg import ObstacleArray

# (name, x0, y0, x1, y1, kind) kind: obs=应检出, struct=墙体类不计虚警, low=低矮不可检出(3线)
GT = [
    ('rackL1_truckL', -14, 9, -6, 15, 'obs'),
    ('rackL2', -14, 6.5, -6, 7.5, 'obs'),
    ('rackL3', -14, 3.5, -6, 4.5, 'obs'),
    ('rackL4', -14, 0.5, -6, 1.5, 'obs'),
    ('rackR1_truckR', 6, 9, 14, 15, 'obs'),
    ('rackR2', 6, 6.5, 14, 7.5, 'obs'),
    ('rackR3', 6, 3.5, 14, 4.5, 'obs'),
    ('rackR4', 6, 0.5, 14, 1.5, 'obs'),
    ('office', -13, -13, -3, -7, 'obs'),
    ('utility', 3, -13, 13, -7, 'obs'),
    ('machine', 6.5, -7.1, 8.5, -5.9, 'obs'),
    ('pallet', -6.6, -7, -5.4, -6, 'skip'),
    ('wall_N', -15.4, 14.9, 15.4, 15.4, 'struct'),
    ('wall_S', -15.4, -15.4, 15.4, -14.9, 'struct'),
    ('wall_E', 14.9, -15.4, 15.4, 15.4, 'struct'),
    ('wall_W', -15.4, -15.4, -14.9, 15.4, 'struct'),
]


class Eval(Node):
    def __init__(self):
        super().__init__('eval_obstacles')
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.dets = []
        self.n_frames = 0
        self.create_subscription(ObstacleArray, '/fp_ob/obstacles', self.cb, 10)

    def cb(self, m):
        self.n_frames += 1
        try:
            tfm = self.buf.lookup_transform('odom', m.header.frame_id,
                                            m.header.stamp, timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            return
        q = tfm.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        t = np.array([tfm.transform.translation.x, tfm.transform.translation.y, 0.0])
        for ob in m.obstacles:
            pts = np.array([[p.x, p.y, 0.0] for p in ob.polygon.points])
            wp = pts @ R.T + t
            self.dets.append({'id': ob.id, 'w': wp, 'c': wp.mean(axis=0)[:2],
                              'bb': (wp[:, 0].min(), wp[:, 1].min(), wp[:, 0].max(), wp[:, 1].max())})


def rect_dist(c, g):
    dx = max(g[1] - c[0], 0.0, c[0] - g[3])
    dy = max(g[2] - c[1], 0.0, c[1] - g[4])
    return float(np.hypot(dx, dy))

def in_zone(det, g, margin=0.6):
    if isinstance(det, dict):
        b = det['bb']
        ox = max(b[0], g[1]) - min(b[2], g[3])
        oy = max(b[1], g[2]) - min(b[3], g[4])
        return ox <= margin and oy <= margin
    return rect_dist(det, g) <= margin


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=15.0)
    args = ap.parse_args()
    rclpy.init()
    n = Eval()
    import time
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(n, timeout_sec=0.1)

    seen = {}
    for d in n.dets:
        key = d['id']
        seen.setdefault(key, d)
    used = set()
    print(f'FRAMES={n.n_frames} UNIQUE_OBSTACLES={len(seen)}')
    for g in GT:
        if g[5] == 'struct':
            continue
        hits = [d for k, d in seen.items() if in_zone(d, g)]
        if g[5] in ('low', 'skip'):
            if g[5] == 'low':
                print(f"GT {g[0]:16s} [低矮-3线不可检出] -> {'误检出!' if hits else '正确未检出'}")
            continue
        if hits:
            for d in hits:
                used.add(d['id'])
            err = min(rect_dist(d['c'], g) for d in hits)
            print(f'GT {g[0]:16s} -> 检出 x{len(hits)} 最近面距离 {err:.2f}m')
        else:
            print(f'GT {g[0]:16s} -> 漏检!')
    fps = [d for k, d in seen.items()
           if k not in used and not any(in_zone(d, g) for g in GT if g[5] == 'struct')]
    print(f'FALSE_POSITIVES={len(fps)}' + (' ' + str([(round(f["c"][0],1),round(f["c"][1],1)) for f in fps]) if fps else ''))


if __name__ == '__main__':
    main()
