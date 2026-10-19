import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from fp_common.transforms import load_calibration, save_calibration, project_points
from fp_common.pointcloud import read_xyz
from fp_common.webview import WebPreview

R_PRESET = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def board_model_3d(bx, by, sq):
    obj = np.zeros((by, bx, 3), np.float32)
    for i in range(by):
        for j in range(bx):
            obj[i, j] = [(j - (bx - 1) / 2) * sq, (i - (by - 1) / 2) * sq, 0]
    return obj.reshape(-1, 3)


def outer_2d(corners, bx, by):
    c = corners.reshape(by, bx, 2)
    dx = c[0, 1] - c[0, 0]
    dy = c[1, 0] - c[0, 0]
    tl = c[0, 0] - (dx + dy) / 2
    tr = c[0, -1] + (dx - dy) / 2
    bl = c[-1, 0] - (dx - dy) / 2
    br = c[-1, -1] + (dx + dy) / 2
    return np.array([tl, tr, br, bl], np.float64)


def fit_plane(pts):
    m = pts.mean(0)
    u, s, vt = np.linalg.svd(pts - m)
    n = vt[2]
    theta = np.arctan2(n[1], n[0]) + np.pi / 2
    ca, sa = np.cos(theta), np.sin(theta)
    a = np.array([ca, sa, 0.0])
    b = np.cross(n, a)
    au = (pts - m) @ a
    av = (pts - m) @ b
    cov = np.cov(np.vstack([au, av]))
    w, evec = np.linalg.eigh(cov)
    e1 = evec[:, np.argmax(w)]
    c2, s2 = e1
    a2 = a * c2 + b * s2
    b2 = -a * s2 + b * c2
    return m, a2, b2, n, (pts - m) @ a2, (pts - m) @ b2


def umeyama(P, Q):
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = mq - R @ mp
    return R, t


def mean_rotation(Rs):
    acc = np.zeros((4, 4))
    for R in Rs:
        w, x, y, z = rot_to_quat(R)
        q = np.array([w, x, y, z])
        acc += np.outer(q, q)
    vals, vecs = np.linalg.eigh(acc)
    q = vecs[:, -1]
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def solve_extrinsics(img, xyz, K, D, R0, t0, bx, by, sq):
    """pure function: one frame -> (R_l2c, t_l2c, res, n_inliers, corners, inl) or None"""
    obj = board_model_3d(bx, by, sq)
    W, H = (bx + 1) * sq, (by + 1) * sq
    model_outers = np.array([[-W / 2, -H / 2, 0], [W / 2, -H / 2, 0],
                             [W / 2, H / 2, 0], [-W / 2, H / 2, 0]])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (bx, by), flags)
    if not ok:
        return None
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    ok2, rvec, tvec = cv2.solvePnP(obj, corners, K, D if len(D) else None,
                                   flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok2:
        return None
    Rcb = cv2.Rodrigues(rvec)[0]
    tcb = tvec.ravel()
    c_l = R0.T @ (tcb - t0)
    d = np.linalg.norm(xyz - c_l, axis=1)
    roi = xyz[d < 0.55]
    if len(roi) < 100:
        return None
    m, a, b, n, au, av = fit_plane(roi)
    inl = roi[(np.abs(au - au.mean()) < W / 2 + 0.03) & (np.abs(av - av.mean()) < H / 2 + 0.03)]
    if len(inl) < 100:
        return None
    m, a, b, n, au, av = fit_plane(inl)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ideal checkerboard is 180-degree symmetric in lidar points -> resolve by
    # choosing the candidate closest to the initial extrinsics guess (R0)
    best = None
    for ns in (n, -n):
        for u in (a, -a, b, -b):
            v = np.cross(ns, u)
            Rlb = np.stack([u, v, ns], axis=1)
            R = Rcb @ Rlb.T
            t = tcb - R @ m
            ang = np.rad2deg(np.arccos(np.clip((np.trace(R.T @ R0) - 1) / 2, -1, 1)))
            if best is None or ang < best[0]:
                best = (ang, R, t)
    res, R_l2c, t_l2c = best
    if res > 12.0:
        return None
    return R_l2c, t_l2c, res, len(inl), corners.reshape(-1, 2), inl



class BoardExtrinsics(Node):
    def __init__(self, args):
        super().__init__('fp_board_extrinsics')
        self.args = args
        calib = load_calibration(args.calib)
        self.K, self.D = calib['K'], calib['D']
        self.R0 = R_PRESET if args.init == 'hesai90' else np.eye(3)
        self.t0 = calib['t'] if args.init == 'hesai90' else np.zeros(3)
        self.obj = board_model_3d(args.bx, args.by, args.sq)
        self.W = args.bx * args.sq
        self.H = args.by * args.sq
        self.model_outers = np.array([[-self.W / 2, -self.H / 2, 0], [self.W / 2, -self.H / 2, 0],
                                      [self.W / 2, self.H / 2, 0], [-self.W / 2, self.H / 2, 0]])
        self.bridge = CvBridge()
        self.img = None
        self.xyz = None
        self.samples = []
        self.last = {}
        self.msg = 'waiting for image + cloud...'
        self.web = WebPreview(args.port, 'fp board extrinsics',
                              buttons=['sample', 'finish', 'quit'], on_button=self._button)
        self.create_subscription(Image, args.image_topic, self._img_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.lidar_topic, self._pcl_cb, qos_profile_sensor_data)
        print(f'>>> open http://localhost:{args.port} <<<')

    def _img_cb(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _pcl_cb(self, msg):
        try:
            self.xyz = read_xyz(msg)
        except Exception:
            pass

    def _solve(self):
        r = solve_extrinsics(self.img, self.xyz, self.K, self.D, self.R0, self.t0,
                             self.args.bx, self.args.by, self.args.sq)
        if r is None:
            return None
        R, t, res, n, corners, inl = r
        uv, dep, _ = project_points(inl, self.K, R, t, self.D)
        return dict(corners=corners.reshape(-1, 1, 2), R=R, t=t, res=res, n=n,
                    uv=uv, dep=dep, inl=inl)

    def _button(self, k):
        if k == 'quit':
            self.web.request_stop()
        elif k == 'sample':
            r = self._solve()
            if r is None:
                self.msg = 'no valid board sample (board not seen / too few lidar pts / bad fit)'
            else:
                self.samples.append(r)
                self.last = r
                self.msg = f'sample {len(self.samples)} added: dev_from_init={r["res"]:.1f}deg n={r["n"]}'
        elif k == 'finish':
            self._finish()

    def _finish(self):
        if not self.samples:
            self.msg = 'no samples'
            return
        Rs = [s['R'] for s in self.samples]
        ts = np.median([s['t'] for s in self.samples], axis=0)
        R = mean_rotation(Rs)
        errs = [s['res'] for s in self.samples]
        save_calibration(self.args.out, self.K, self.D, R, ts,
                         (self.img.shape[1], self.img.shape[0]), note='board_extrinsics result')
        self.msg = (f'saved {self.args.out}: {len(self.samples)} samples, '
                    f'mean dev-from-init {np.mean(errs):.1f}deg')
        self.get_logger().info(self.msg)

    def _render(self):
        canvas = self.img.copy()
        if self.last.get('corners') is not None:
            cv2.drawChessboardCorners(canvas, (self.args.bx, self.args.by), self.last['corners'], True)
        if self.last.get('uv') is not None:
            for (u, v), dp in zip(self.last['uv'], self.last['dep']):
                if dp > 0.1 and 0 <= u < canvas.shape[1] and 0 <= v < canvas.shape[0]:
                    cv2.circle(canvas, (int(u), int(v)), 1, (0, 255, 0), -1)
        cv2.putText(canvas, f"samples={len(self.samples)}  {self.msg[:100]}", (20, 30),
                    0, 0.7, (0, 200, 255), 2)
        return canvas

    def run(self):
        i = 0
        while rclpy.ok() and not self.web.stopped:
            rclpy.spin_once(self, timeout_sec=0.02)
            i += 1
            if self.img is None or self.xyz is None or len(self.xyz) == 0:
                continue
            if i % 10 == 0:
                r = self._solve()
                if r is not None:
                    self.last = r
                    self.web.status = (f'board FOUND, lidar inliers={r["n"]}, corner-residual='
                                       f'{r["res"]:.1f}deg  -> press [sample] to record, '
                                       f'move board to new poses, then [finish]\n{self.msg}')
                else:
                    self.web.status = f'board not ready: {self.msg}'
            self.web.set_image(self._render())
        self.web.shutdown()


def main():
    ap = argparse.ArgumentParser(description='chessboard-based LiDAR-camera extrinsics solver')
    ap.add_argument('--calib', required=True, help='yaml with K,D (intrinsics)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--init', default='hesai90', choices=['hesai90', 'identity'])
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--lidar-topic', default='/hesai/points')
    ap.add_argument('--port', type=int, default=8092)
    ap.add_argument('--board', default='11x8')
    ap.add_argument('--square', type=float, default=0.035)
    args = ap.parse_args()
    args.bx, args.by = (int(v) for v in args.board.lower().split('x'))
    args.sq = args.square
    rclpy.init()
    node = BoardExtrinsics(args)
    node.run()
    node.destroy_node()
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
