import argparse
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from fp_common.transforms import (load_calibration, save_calibration,
                                  project_points, in_image_mask, euler_to_R)
from fp_common.pointcloud import read_xyz
from fp_common.webview import WebPreview

SLIDERS = {
    'dx[mm]': dict(min=-2000, max=2000, step=1, init=0),
    'dy[mm]': dict(min=-2000, max=2000, step=1, init=0),
    'dz[mm]': dict(min=-2000, max=2000, step=1, init=0),
    'rx[deg]': dict(min=-180, max=180, step=0.1, init=0),
    'ry[deg]': dict(min=-180, max=180, step=0.1, init=0),
    'rz[deg]': dict(min=-180, max=180, step=0.1, init=0),
}
BUTTONS = ['init:hesai90', 'init:identity', 'save', 'reset', 'pick:on/off', 'pnp', 'clear', 'quit']
# Hesai/Pandar convention: x fwd, y left, z up -> camera: x right, y down, z fwd
R_PRESET_L2C = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


class AlignTool(Node):
    def __init__(self, args):
        super().__init__('fp_manual_align')
        self.args = args
        calib = load_calibration(args.calib)
        self.K, self.D = calib['K'], calib['D']
        self.R0, self.t0 = calib['R'].copy(), calib['t'].copy()
        self.bridge = CvBridge()
        self.img = None
        self.xyz = None
        self.pairs = []
        self.pick_mode = False
        self.proj_cache = None
        self.n_drawn = 0
        self.msg = ''

        self.create_subscription(Image, args.image_topic, self._img_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.lidar_topic, self._pcl_cb, qos_profile_sensor_data)

        self.web = WebPreview(args.port, 'fp lidar-camera align', sliders=SLIDERS,
                              buttons=BUTTONS, on_button=self._button, on_click=self._click)
        print(f'>>> open http://localhost:{args.port}  (sliders=coarse, pick+pnp=refine) <<<')

    def _img_cb(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'img: {e}')

    def _pcl_cb(self, msg):
        try:
            self.xyz = read_xyz(msg)
        except Exception as e:
            self.get_logger().warn(f'pcl: {e}')

    def _current_rt(self):
        p = self.web.params
        td = np.array([p['dx[mm]'], p['dy[mm]'], p['dz[mm]']]) / 1000.0
        Rd = euler_to_R(p['rx[deg]'], p['ry[deg]'], p['rz[deg]'])
        return Rd @ self.R0, Rd @ self.t0 + td

    def _button(self, k):
        if k == 'quit':
            self.web.request_stop()
        elif k == 'init:hesai90':
            self.R0 = R_PRESET_L2C.copy()
            self.web.set_params({s: 0 for s in SLIDERS})
            self.msg = 'R0 <- standard LiDAR(x-fwd,y-left,z-up)->Cam preset, t unchanged. Now drag sliders.'
        elif k == 'init:identity':
            self.R0 = np.eye(3)
            self.web.set_params({s: 0 for s in SLIDERS})
            self.msg = 'R0 <- identity'
        elif k == 'reset':
            self.web.set_params({s: 0 for s in SLIDERS})
        elif k == 'save':
            R, t = self._current_rt()
            self.R0, self.t0 = R, t
            save_calibration(self.args.out, self.K, self.D, R, t,
                             (self.img.shape[1], self.img.shape[0]) if self.img is not None else (0, 0),
                             note='manual_align result')
            self.web.set_params({s: 0 for s in SLIDERS})
            self.msg = f'saved -> {self.args.out}'
            self.get_logger().info(self.msg)
        elif k.startswith('pick'):
            self.pick_mode = not self.pick_mode
            self.msg = f'pick mode {"ON: click targets in image" if self.pick_mode else "OFF"}'
        elif k == 'clear':
            self.pairs.clear()
            self.msg = 'pairs cleared'
        elif k == 'pnp':
            self._pnp()

    def _click(self, x, y):
        if not self.pick_mode or self.proj_cache is None:
            return
        uv, pts = self.proj_cache
        d2 = (uv[:, 0] - x) ** 2 + (uv[:, 1] - y) ** 2
        j = int(np.argmin(d2))
        if d2[j] ** 0.5 <= 20.0:
            self.pairs.append((pts[j].astype(np.float64), np.array([x, y], np.float64)))
            self.msg = f'pair {len(self.pairs)} added'

    def _pnp(self):
        if len(self.pairs) < 6:
            self.msg = 'PnP needs >=6 pairs (ideally 8-12, non-coplanar)'
            return
        obj = np.array([p for p, _ in self.pairs], np.float64)
        img_pts = np.array([q for _, q in self.pairs], np.float64)
        centered = obj - obj.mean(0)
        sv = np.linalg.svd(centered, compute_uv=False)
        coplanar = sv[2] < 0.1 * sv[0]
        Rg, tg = self._current_rt()
        guess = np.hstack([Rg, tg.reshape(3, 1)])
        ok, rvec, tvec = cv2.solvePnP(
            obj, img_pts, self.K, self.D if len(self.D) else None,
            rvec=cv2.Rodrigues(Rg)[0], tvec=tg.reshape(3, 1),
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            self.msg = 'PnP failed'
            return
        self.R0 = cv2.Rodrigues(rvec)[0]
        self.t0 = tvec.ravel()
        self.web.set_params({s: 0 for s in SLIDERS})
        errs = project_points(obj, self.K, self.R0, self.t0, self.D)[0]
        note = ' [coplanar: refined from coarse guess]' if coplanar else ''
        self.msg = (f'PnP done, mean reproj err = {np.linalg.norm(errs - img_pts, axis=1).mean():.2f}px{note} '
                    f'-> press [save]')
        self.get_logger().info(self.msg)

    def _render(self):
        R, t = self._current_rt()
        P = self.xyz[::self.args.stride]
        d = np.linalg.norm(P, axis=1)
        P = P[(d > self.args.min_range) & (d < self.args.max_range)]
        uv, depth, _ = project_points(P, self.K, R, t, self.D)
        vis = in_image_mask(uv, depth, self.img.shape[1], self.img.shape[0])
        uv, P = uv[vis], P[vis]
        self.proj_cache = (uv, P)
        self.n_drawn = len(uv)
        canvas = self.img.copy()
        ds = max(1, self.args.draw_stride)
        zmin, zmax = 0.5, 10.0
        for (u, v), p in zip(uv[::ds], P[::ds]):
            z = float(np.clip((p[2] - zmin) / (zmax - zmin), 0, 1))
            color = (int(255 * z), 80, int(255 * (1 - z)))
            cv2.circle(canvas, (int(u), int(v)), 2, color, -1)
        for i, (p3, (u, v)) in enumerate(self.pairs):
            cv2.circle(canvas, (int(u), int(v)), 6, (0, 255, 255), 2)
            cv2.putText(canvas, str(i + 1), (int(u) + 7, int(v) - 7), 0, 0.6, (0, 255, 255), 2)
            su, sd, _ = project_points(p3[None, :], self.K, R, t, self.D)
            if sd[0] > 0.05:
                cv2.drawMarker(canvas, (int(su[0, 0]), int(su[0, 1])), (255, 255, 0),
                               cv2.MARKER_TILTED_CROSS, 10, 2)
        mode = 'PICK: click same targets in image' if self.pick_mode else 'coarse-align with sliders'
        p = self.web.params
        head = (f"{mode}  pairs={len(self.pairs)}  "
                f"t=({p['dx[mm]']:.0f},{p['dy[mm]']:.0f},{p['dz[mm]']:.0f})mm "
                f"r=({p['rx[deg]']:.1f},{p['ry[deg]']:.1f},{p['rz[deg]']:.1f})deg")
        cv2.putText(canvas, head, (20, 30), 0, 0.8, (255, 255, 255), 2)
        if self.msg:
            cv2.putText(canvas, self.msg[:110], (20, 60), 0, 0.7, (0, 200, 255), 2)
        return canvas

    def run(self):
        self.web.status = 'waiting for image + pointcloud...'
        while rclpy.ok() and not self.web.stopped:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.img is None or self.xyz is None or len(self.xyz) == 0:
                continue
            p = self.web.params
            self.web.status = (
                f"mode={'PICK: click the SAME physical corner in the image (board corners, box edges)' if self.pick_mode else 'COARSE: drag sliders until overlay (colored dots) sits on real edges'}\n"
                f"pairs={len(self.pairs)}  t=({p['dx[mm]']:.0f},{p['dy[mm]']:.0f},{p['dz[mm]']:.0f})mm"
                f"  r=({p['rx[deg]']:.1f},{p['ry[deg]']:.1f},{p['rz[deg]']:.1f})deg"
                f"  |  drawn pts={self.n_drawn}  |  flow: coarse -> pick:on/off -> click >=8 -> pnp -> save -> quit\n{self.msg}")
            if self.n_drawn == 0:
                self.web.status += '\n!! no overlay points in image: extrinsics way off -> press [init:hesai90] first'
            self.web.set_image(self._render())
        self.web.shutdown()


def main():
    ap = argparse.ArgumentParser(description='LiDAR-Camera alignment via browser UI (coarse sliders + PnP refine)')
    ap.add_argument('--calib', required=True, help='yaml with K,D and coarse (or identity) R,t')
    ap.add_argument('--out', required=True, help='output extrinsics yaml')
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--lidar-topic', default='/hesai/points')
    ap.add_argument('--port', type=int, default=8091)
    ap.add_argument('--stride', type=int, default=4, help='point sub-sample for projection')
    ap.add_argument('--draw-stride', type=int, default=3, help='sub-sample for drawing')
    ap.add_argument('--min-range', type=float, default=0.5)
    ap.add_argument('--max-range', type=float, default=30.0)
    args = ap.parse_args()
    rclpy.init()
    gui = AlignTool(args)
    gui.run()
    gui.destroy_node()
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
