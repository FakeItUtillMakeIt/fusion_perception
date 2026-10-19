import cv2
import numpy as np
import rclpy
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from fp_common.transforms import load_calibration, project_points, in_image_mask
from fp_common.pointcloud import read_xyz_label
from fp_common.policy import Policy


class OverlayNode(Node):
    def __init__(self):
        super().__init__('fp_overlay')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('labeled_topic', '/fp/labeled_points')
        self.declare_parameter('calib_path', '')
        self.declare_parameter('policy_path', '')
        self.declare_parameter('alpha', 0.45)
        self.declare_parameter('splat_px', 9)
        self.declare_parameter('stride', 2)
        self.declare_parameter('topic', '/fp/overlay')

        calib = load_calibration(self.get_parameter('calib_path').value)
        self.K, self.D = calib['K'], calib['D']
        self.R, self.t = calib['R'], calib['t']
        self.policy = Policy(self.get_parameter('policy_path').value)
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.get_parameter('topic').value, 5)
        sub_img = message_filters.Subscriber(self, Image, self.get_parameter('image_topic').value)
        sub_pcl = message_filters.Subscriber(self, PointCloud2, self.get_parameter('labeled_topic').value)
        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_img, sub_pcl], 50, 0.15)
        sync.registerCallback(self._cb)

    def _cb(self, img_msg, cloud_msg):
        alpha = float(self.get_parameter('alpha').value)
        splat = int(self.get_parameter('splat_px').value)
        stride = max(1, int(self.get_parameter('stride').value))
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        xyz, labels = read_xyz_label(cloud_msg)
        uv, depth, _ = project_points(xyz.astype(np.float64), self.K, self.R, self.t, self.D)
        valid = in_image_mask(uv, depth, img.shape[1], img.shape[0])
        uv, labels = uv[valid][::stride], labels[valid][::stride]

        canvas = np.zeros_like(img)
        mask = np.zeros(img.shape[:2], np.float32)
        colors = self.policy.bgr_of_labels(labels)
        for (u, v), c in zip(uv, colors):
            cv2.circle(canvas, (int(u), int(v)), max(1, splat // 2), tuple(int(x) for x in c), -1)
            cv2.circle(mask, (int(u), int(v)), max(1, splat // 2), 1.0, -1)
        if splat > 3:
            k = (splat | 1) * 2 + 1
            canvas = cv2.GaussianBlur(canvas, (k, k), 0)
            mask = cv2.GaussianBlur(mask, (k, k), 0)
        m = np.clip(mask * 1.6, 0, 1)[..., None] * alpha
        out = (img * (1 - m) + canvas * m).astype(np.uint8)

        cv2.putText(out, 'passable', (20, 30), 0, 0.7, (0, 200, 0), 2)
        cv2.putText(out, 'soft', (150, 30), 0, 0.7, (0, 220, 220), 2)
        cv2.putText(out, 'hard', (250, 30), 0, 0.7, (0, 0, 230), 2)
        cv2.putText(out, 'transparent', (330, 30), 0, 0.7, (255, 0, 255), 2)

        vis = self.bridge.cv2_to_imgmsg(out, 'bgr8')
        vis.header = img_msg.header
        self.pub.publish(vis)


def main():
    rclpy.init()
    node = OverlayNode()
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
