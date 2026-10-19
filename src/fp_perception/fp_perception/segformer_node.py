import time

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation


class SegformerNode(Node):
    def __init__(self):
        super().__init__('fp_segformer')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_id', 'nvidia/segformer-b2-finetuned-ade-512-512')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('half', True)
        self.declare_parameter('every_n', 1)
        self.declare_parameter('labels_topic', '/fp/labels')
        self.declare_parameter('vis_topic', '/fp/sem_vis')
        self.declare_parameter('policy_path', '')
        self.declare_parameter('smooth_k', 3)

        model_id = self.get_parameter('model_id').value
        self.device = torch.device(self.get_parameter('device').value)
        half = bool(self.get_parameter('half').value) and self.device.type == 'cuda'

        self.get_logger().info(f'loading {model_id} on {self.device}')
        self.processor = AutoImageProcessor.from_pretrained(model_id)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_id)
        self.model.to(self.device)
        self.model.eval()
        if half:
            self.model.half()
        ip = getattr(self.processor, 'image_processor', self.processor)
        size = getattr(ip, 'size', None) or {}
        self.inp_w = int(size.get('width', 512))
        self.inp_h = int(size.get('height', 512))
        self.mean = np.array(getattr(ip, 'image_mean', [0.5] * 3), np.float32)
        self.std = np.array(getattr(ip, 'image_std', [0.5] * 3), np.float32)
        self.dtype = torch.float16 if half else torch.float32
        self.get_logger().info(f'model ready, {len(self.model.config.id2label)} classes, '
                               f'input {self.inp_w}x{self.inp_h}')

        self.bridge = CvBridge()
        self._n = 0
        self._t0 = time.time()
        self._count = 0
        rng = np.random.default_rng(42)
        self.lut = rng.integers(0, 255, size=(256, 3), dtype=np.uint8)
        self.lut[255] = (60, 60, 60)
        policy_path = self.get_parameter('policy_path').value
        if policy_path:
            from fp_common.policy import Policy
            pol = Policy(policy_path, id2label=self.model.config.id2label)
            for lid in range(256):
                if pol.group_lut[lid] >= 0:
                    g = pol.group_order[pol.group_lut[lid]]
                    self.lut[lid] = pol.colors[g]
            self.get_logger().info('sem_vis uses policy colors for mapped classes')

        self.pub_labels = self.create_publisher(Image, self.get_parameter('labels_topic').value, 5)
        self.pub_vis = self.create_publisher(Image, self.get_parameter('vis_topic').value, 5)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._cb, qos_profile_sensor_data)

        self._buf = []

    def _cb(self, msg):
        if self._n % int(self.get_parameter('every_n').value) != 0:
            self._n += 1
            return
        self._n += 1
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        small = cv2.resize(bgr, (self.inp_w, self.inp_h), interpolation=cv2.INTER_LINEAR)
        x = small.astype(np.float32)[:, :, ::-1] / 255.0
        x = (x - self.mean) / self.std
        x = np.ascontiguousarray(x.transpose(2, 0, 1)[None])
        k = int(self.get_parameter('smooth_k').value)
        with torch.no_grad():
            pv = torch.from_numpy(x).to(self.device, self.dtype)
            logits = self.model(pixel_values=pv).logits.float()
            if k > 1:
                self._buf.append(logits[0].half().cpu())
                if len(self._buf) > k:
                    self._buf.pop(0)
                logits = torch.stack(self._buf).mean(0, keepdim=True)
            low = logits.argmax(1)[0].to(torch.uint8).cpu().numpy()
        labels = cv2.resize(low, (msg.width, msg.height), interpolation=cv2.INTER_NEAREST)

        hmsg = Image(header=msg.header, encoding='mono8', height=msg.height, width=msg.width,
                     is_bigendian=0, step=msg.width, data=labels.tobytes())
        self.pub_labels.publish(hmsg)

        vis = self.bridge.cv2_to_imgmsg(self.lut[labels], encoding='bgr8')
        vis.header = msg.header
        self.pub_vis.publish(vis)

        self._count += 1
        if self._count % 30 == 0:
            dt = time.time() - self._t0
            self.get_logger().info(f'inference ~{self._count / max(dt, 1e-6):.1f} fps')
            self._t0, self._count = time.time(), 0


def main():
    rclpy.init()
    node = SegformerNode()
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
