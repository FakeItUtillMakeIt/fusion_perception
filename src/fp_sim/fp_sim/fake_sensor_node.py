import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge

from fp_common.pointcloud import make_cloud, pack_rgb
from fp_common.transforms import save_calibration
from fp_sim.scene import COLORS, SimScene

K_DEFAULT = [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
R_L2C = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
T_L2C = [0.05, -0.05, 0.12]


def rot_to_quat(R):
    R = np.asarray(R, float)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
        w = (R[k, j] - R[j, k]) / s
        x = 0.25 * s if i == 0 else (R[j, 0] + R[0, j]) / s
        y = 0.25 * s if i == 1 else (R[k, 1] + R[1, k]) / s
        z = 0.25 * s if i == 2 else (R[k, 2] + R[2, k]) / s
    return w, x, y, z


class FakeSensorNode(Node):
    def __init__(self):
        super().__init__('fp_fake_sensor')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('lidar_topic', '/hesai/points')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('lidar_hz', 10.0)
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('truth_out', '')
        self.declare_parameter('board_motion', True)
        self.declare_parameter('publish_image', True)

        self.W = int(self.get_parameter('width').value)
        self.H = int(self.get_parameter('height').value)
        self.K = np.array(K_DEFAULT, float)
        self.scene = SimScene(R_L2C, T_L2C)
        self.bridge = CvBridge()
        self.max_range = float(self.get_parameter('max_range').value)
        self.lidar_every = max(1, int(round(30.0 / float(self.get_parameter('lidar_hz').value))))
        self._i = 0
        self._t0 = time.time()

        self.pub_img = self.create_publisher(Image, self.get_parameter('image_topic').value, 10)
        self.pub_pcl = self.create_publisher(PointCloud2, self.get_parameter('lidar_topic').value, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.get_parameter('info_topic').value, 10)

        tfb = StaticTransformBroadcaster(self)
        # ROS convention: odom = x forward, y left, z up -> align with lidar frame
        # NOTE: send both in ONE call -- transient_local history depth is 1,
        # a second call would overwrite the first for late subscribers
        Rc = np.array(R_L2C).T
        tc = -Rc @ np.array(T_L2C)
        tfb.sendTransform([self._tf('odom', 'lidar_link', np.eye(3), np.zeros(3)),
                           self._tf('odom', 'camera_link', Rc, tc)])

        truth = self.get_parameter('truth_out').value
        if truth:
            save_calibration(truth, self.K, [], np.array(R_L2C), np.array(T_L2C), (self.W, self.H),
                             note='fp_sim ground truth')
            self.get_logger().info(f'ground truth written to {truth}')

        self.create_timer(1.0 / 30.0, self._tick)

    def _tf(self, parent, child, R, t):
        m = TransformStamped()
        m.header.frame_id = parent
        m.child_frame_id = child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
        w, x, y, z = rot_to_quat(R)
        m.transform.rotation.w, m.transform.rotation.x = w, x
        m.transform.rotation.y, m.transform.rotation.z = y, z
        return m

    def _tick(self):
        ts = time.time() - self._t0
        if not self.get_parameter('board_motion').value:
            ts = 0.0
        now = self.get_clock().now().to_msg()
        publish_image = bool(self.get_parameter('publish_image').value)
        img = self.scene.render(self.K, self.W, self.H, ts) if publish_image else None
        if publish_image:
            m = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            m.header.frame_id = 'camera_link'
            m.header.stamp = now
            self.pub_img.publish(m)
            info = CameraInfo(header=m.header, width=self.W, height=self.H,
                              k=[float(x) for x in self.K.ravel()],
                              distortion_model='plumb_bob', d=[0.0] * 5)
            self.pub_info.publish(info)
        else:
            m = None

        self._i += 1
        if self._i % self.lidar_every != 0:
            return
        pts, labels = self.scene.scan(ts, max_range=self.max_range)
        c = np.array([COLORS.get(int(l), (128, 128, 128)) for l in labels], np.uint8)
        rgb = pack_rgb(c[:, 2], c[:, 1], c[:, 0])
        hdr = Header(stamp=now, frame_id='lidar_link')
        cloud = make_cloud(hdr, pts, extra=[
            ('label', PointField.UINT8, labels),
            ('rgb', PointField.FLOAT32, rgb)])
        self.pub_pcl.publish(cloud)


def main():
    rclpy.init()
    node = FakeSensorNode()
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
