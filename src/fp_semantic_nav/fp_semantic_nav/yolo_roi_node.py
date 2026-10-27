#!/usr/bin/env python3
"""YOLO-World ROI 节点：文本提示检测 -> 图像框 -> /fp_sem/manual_roi。

与 glass_detector 的 ROI 接口完全一致（PolygonStamped，图像像素四角），
替换手动 pub 即为正式链路；后续换微调过的封闭权重只需改 model 参数。

用法：
  ros2 run fp_semantic_nav yolo_roi_node
  话题: 入 /camera/image_raw  出 /fp_sem/manual_roi, /fp_sem/yolo_vis
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
import cv2
from ultralytics import YOLOWorld


class YoloRoiNode(Node):
    def __init__(self):
        super().__init__('yolo_roi')
        d = self.declare_parameter
        self.model_path = d('model', 'yolov8s-worldv2.pt').value
        self.classes = list(d('text_classes', ['glass panel', 'glass door', 'glass wall']).value)
        self.conf = d('conf_thresh', 0.15).value
        self.every_n = d('every_n', 2).value
        self.min_w = d('min_box_w', 60).value
        self.max_w_frac = d('max_box_width_frac', 0.8).value
        self.out = self.create_publisher(PolygonStamped, '/fp_sem/manual_roi', 10)
        self.vis = self.create_publisher(Image, '/fp_sem/yolo_vis', 5)
        self.model = YOLOWorld(self.model_path)
        self.model.set_classes(self.classes)
        self.n = 0
        self.img_w = 640
        self._t0 = self.get_clock().now()
        self._cnt = 0
        self.create_subscription(Image, d('image_topic', '/camera/image_raw').value,
                                 self.cb, qos_profile_sensor_data)
        self.get_logger().info('yolo_roi up: %s classes=%s conf=%.2f' % (self.model_path, self.classes, self.conf))

    def cb(self, msg):
        self.n += 1
        if self.n % self.every_n:
            return
        self.img_w = msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, msg.step // msg.width)
        res = self.model.predict(img, conf=self.conf, verbose=False)[0]
        boxes = getattr(res, 'boxes', None)
        best = None
        if boxes is not None and len(boxes):
            xy = boxes.xyxy.cpu().numpy()
            areas = (xy[:, 2] - xy[:, 0]) * (xy[:, 3] - xy[:, 1])
            i = int(areas.argmax())
            if self.min_w <= (xy[i][2] - xy[i][0]) <= self.img_w * self.max_w_frac:
                x0, y0, x1, y1 = xy[i]
                cls = int(boxes.cls[i])
                name = res.names[cls]
                best = (x0, y0, x1, y1, name, float(boxes.conf[i]))
        canvas = img.copy()
        if best:
            x0, y0, x1, y1, name, c = best
            cv2.rectangle(canvas, (int(x0), int(y0)), (int(x1), int(y1)), (255, 0, 255), 2)
            cv2.putText(canvas, '%s %.2f' % (name, c), (int(x0), int(y0) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            m = PolygonStamped()
            m.header = msg.header
            m.header.frame_id = 'camera_link'
            v = y1
            m.polygon.points = [Point32(x=float(x0), y=float(y0), z=0.0), Point32(x=float(x1), y=float(y0), z=0.0),
                                Point32(x=float(x1), y=float(v), z=0.0), Point32(x=float(x0), y=float(v), z=0.0)]
            self.out.publish(m)
            self._cnt += 1
            if self._cnt % 20 == 1:
                self.get_logger().info('hit: %s conf=%.2f box=(%.0f,%.0f,%.0f,%.0f)' % (name, c, x0, y0, x1, y1))
        out = self.bridge_encode(canvas, msg)
        self.vis.publish(out)

    def bridge_encode(self, arr, src):
        m = Image()
        m.header = src.header
        m.height, m.width = arr.shape[:2]
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = m.width * 3
        m.data = np.ascontiguousarray(arr).tobytes()
        return m


def main():
    rclpy.init()
    n = YoloRoiNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
