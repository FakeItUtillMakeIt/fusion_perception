import argparse
import os
import sys

import cv2
import numpy as np


def parse_board(s):
    w, h = s.lower().split('x')
    return int(w), int(h)


def capture(topic, out_dir, board=None, square=0.025, out=None, port=8090, node_name='fp_capture'):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from fp_common.webview import WebPreview

    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Node(node_name)
    bridge = CvBridge()
    state = {'img': None, 'count': 0}

    def cb(msg):
        try:
            state['img'] = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            state['img'] = None

    node.create_subscription(Image, topic, cb, qos_profile_sensor_data)

    def on_button(k):
        if k == 'grab' and state['img'] is not None:
            path = os.path.join(out_dir, f'{state["count"]:04d}.png')
            cv2.imwrite(path, state['img'])
            state['count'] += 1
            print('saved', path)
        elif k == 'quit':
            web.request_stop()

    web = WebPreview(port, 'fp camera intrinsics capture',
                     buttons=['grab', 'quit'], on_button=on_button)
    print(f'saving frames to {out_dir}, topic={topic}')
    print(f'>>> open http://localhost:{port} in your browser <<<')
    hint = 'no image yet: check topic / camera driver'
    try:
        while rclpy.ok() and not web.stopped:
            rclpy.spin_once(node, timeout_sec=0.05)
            img = state['img']
            if img is None:
                preview = np.full((480, 640, 3), 40, np.uint8)
                cv2.putText(preview, hint, (20, 240), 0, 0.8, (0, 200, 255), 2)
            else:
                preview = img
                web.status = f"saved: {state['count']}   [grab] to save current frame, [quit] to finish"
            web.set_image(preview)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print('capture loop:', e)
    finally:
        web.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    if board and out and state['count'] > 0:
        solve(out_dir, parse_board(board), square, out)


def solve(img_dir, board_size, square_m, out_yaml):
    pattern = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    pattern[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_m
    objp, corners_all, imgs_used = [], [], []
    files = sorted(f for f in os.listdir(img_dir) if f.endswith(('.png', '.jpg')))
    for i, f in enumerate(files):
        img = cv2.imread(os.path.join(img_dir, f))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, board_size, flags)
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objp.append(pattern)
            corners_all.append(corners)
            imgs_used.append((f, img.shape[1], img.shape[0]))
            print(f'  [ok] {f}')
        else:
            print(f'  [skip] {f}')
    if len(objp) < 6:
        print(f'not enough views: {len(objp)} (need >=6)')
        sys.exit(1)
    w, h = imgs_used[0][1], imgs_used[0][2]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objp, corners_all, (w, h), None, None)
    print(f'views={len(objp)} size={w}x{h} rms={rms:.4f}')
    print('K =', np.round(K, 4).tolist())
    print('D =', np.round(D.ravel(), 6).tolist())
    _write_yaml(out_yaml, K, D, (w, h))
    print('saved', out_yaml)


def _write_yaml(path, K, D, size):
    import yaml as y
    d = {
        'image_width': int(size[0]), 'image_height': int(size[1]),
        'K': [float(x) for x in K.ravel()],
        'D': [float(x) for x in np.asarray(D).ravel()],
        'R_lidar_to_cam': [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
        't_lidar_to_cam': [0.0, 0.0, 0.0],
        'note': 'intrinsics solved by fp_calibration.camera_intrinsics; extrinsics TBD',
    }
    with open(path, 'w') as f:
        y.safe_dump(d, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser(description='camera intrinsics: capture frames from a ROS 2 image topic and/or solve offline')
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--topic', default='/camera/image_raw')
    c.add_argument('--dir', default='./intrinsics_frames')
    c.add_argument('--board', default=None, help='inner corners WxH; if set, solve right after capture')
    c.add_argument('--square', type=float, default=0.025, help='square size in meters')
    c.add_argument('--out', default='camera_calib.yaml')
    c.add_argument('--port', type=int, default=8090)
    s = sub.add_parser('solve')
    s.add_argument('--dir', default='./intrinsics_frames')
    s.add_argument('--board', default='11x8', help='inner corners WxH')
    s.add_argument('--square', type=float, default=0.025, help='square size in meters')
    s.add_argument('--out', default='camera_calib.yaml')
    a = ap.parse_args()
    if a.cmd == 'capture':
        capture(a.topic, a.dir, board=a.board, square=a.square, out=a.out, port=a.port)
    else:
        solve(a.dir, parse_board(a.board), a.square, a.out)


if __name__ == '__main__':
    main()
