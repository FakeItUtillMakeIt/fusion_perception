import numpy as np
import rclpy
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField

from fp_common.transforms import load_calibration, project_points, in_image_mask
from fp_common.pointcloud import read_xyz, make_cloud, pack_rgb
from fp_common.policy import Policy


class LidarLabelNode(Node):
    def __init__(self):
        super().__init__('fp_lidar_label')
        self.declare_parameter('calib_path', '')
        self.declare_parameter('policy_path', '')
        self.declare_parameter('lidar_topic', '/hesai/points')
        self.declare_parameter('labels_topic', '/fp/labels')
        self.declare_parameter('slo', 0.05)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 50.0)

        calib = load_calibration(self.get_parameter('calib_path').value)
        self.K, self.D = calib['K'], calib['D']
        self.R, self.t = calib['R'], calib['t']
        self.policy = Policy(self.get_parameter('policy_path').value)

        self.pub_labeled = self.create_publisher(PointCloud2, '/fp/labeled_points', 5)
        self.pub_color = self.create_publisher(PointCloud2, '/fp/points_policy', 5)
        self.pub_filtered = self.create_publisher(PointCloud2, '/fp/filtered_points', 5)

        sub_img = message_filters.Subscriber(self, Image, self.get_parameter('labels_topic').value)
        sub_pcl = message_filters.Subscriber(self, PointCloud2, self.get_parameter('lidar_topic').value)
        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_pcl, sub_img], 5, float(self.get_parameter('slo').value))
        sync.registerCallback(self._cb)

    def _cb(self, cloud_msg, labels_msg):
        xyz = read_xyz(cloud_msg)
        n = len(xyz)
        if n == 0:
            return
        labels = np.frombuffer(labels_msg.data, dtype=np.uint8).reshape(
            labels_msg.height, labels_msg.step)[:, :labels_msg.width]
        lab_h, lab_w = labels.shape

        uv, depth, _ = project_points(xyz.astype(np.float64), self.K, self.R, self.t, self.D)
        valid = in_image_mask(uv, depth, lab_w, lab_h)
        rmin = float(self.get_parameter('min_range').value)
        rmax = float(self.get_parameter('max_range').value)
        valid &= (depth > rmin) & (depth < rmax)

        out_labels = np.full(n, 255, dtype=np.uint8)
        vi = np.nonzero(valid)[0]
        u = np.clip(uv[vi, 0].astype(np.int32), 0, lab_w - 1)
        v = np.clip(uv[vi, 1].astype(np.int32), 0, lab_h - 1)
        out_labels[vi] = labels[v, u]

        hdr = cloud_msg.header
        self.pub_labeled.publish(make_cloud(
            hdr, xyz, extra=[('label', PointField.UINT8, out_labels)]))

        bgr = self.policy.bgr_of_labels(out_labels)
        rgb = self.pack_rgb_np(bgr)
        self.pub_color.publish(make_cloud(hdr, xyz, extra=[('rgb', PointField.FLOAT32, rgb)]))

        drop = self.policy.drop_mask(out_labels)
        keep = ~drop
        self.pub_filtered.publish(make_cloud(hdr, xyz[keep]))

    @staticmethod
    def pack_rgb_np(bgr):
        return pack_rgb(bgr[..., 2].ravel(), bgr[..., 1].ravel(), bgr[..., 0].ravel())


def main():
    rclpy.init()
    node = LidarLabelNode()
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
