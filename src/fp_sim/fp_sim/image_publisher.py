import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class ImagePub(Node):
    def __init__(self, path, topic, rate, loop):
        super().__init__('fp_image_pub')
        self.bridge = CvBridge()
        self.loop = loop
        self.img = cv2.imread(path)
        self.is_video = self.img is None
        if self.is_video:
            self.cap = cv2.VideoCapture(path)
            if not self.cap.isOpened():
                raise SystemExit(f'cannot read {path} as image or video')
            fps = self.cap.get(cv2.CAP_PROP_FPS) or 30
            self.period = 1.0 / (rate if rate > 0 else fps)
            self.get_logger().info(f'publishing VIDEO {path} @ {1/self.period:.1f}Hz -> {topic}')
        else:
            self.period = 1.0 / (rate if rate > 0 else 10)
            self.get_logger().info(f'publishing IMAGE {path} ({self.img.shape[1]}x{self.img.shape[0]}) @ {1/self.period:.1f}Hz -> {topic}')
        self.pub = self.create_publisher(Image, topic, 10)
        self.create_timer(self.period, self._tick)

    def _tick(self):
        if self.is_video:
            ok, frame = self.cap.read()
            if not ok:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = self.cap.read()
                if not ok:
                    return
        else:
            frame = self.img
        m = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        m.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_link')
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description='publish an image or video as Image topic (for segformer sanity checks)')
    ap.add_argument('--path', required=True, help='jpg/png or mp4/avi video')
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--rate', type=float, default=0.0, help='Hz; 0 = native video fps or 10Hz for image')
    ap.add_argument('--once', action='store_true', help='do not loop video')
    args = ap.parse_args()
    rclpy.init()
    node = ImagePub(args.path, args.topic, args.rate, not args.once)
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
