import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__file__)))
from fp_common.ade20k import ID2LABEL


class Probe(Node):
    def __init__(self, topic, region):
        super().__init__('fp_label_probe')
        self.region = region
        self.latest = None
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, m):
        lab = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
        x0, y0, x1, y1 = self.region
        h, w = lab.shape
        crop = lab[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)]
        self.latest = crop


def main():
    ap = argparse.ArgumentParser(description='print dominant classes of a region of /fp/labels')
    ap.add_argument('--topic', default='/fp/labels')
    ap.add_argument('--region', default='0.3,0.75,0.7,1.0',
                    help='x0,y0,x1,y1 fractions; default = bottom-center (usually the ground)')
    a = ap.parse_args()
    x0, y0, x1, y1 = (float(v) for v in a.region.split(','))
    rclpy.init()
    node = Probe(a.topic, (x0, y0, x1, y1))
    t0 = time.time()
    while time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        print('no messages on', a.topic)
        return
    u, c = np.unique(node.latest, return_counts=True)
    order = np.argsort(-c)
    print(f'region x[{x0},{x1}] y[{y0},{y1}] dominant classes:')
    for i in order[:8]:
        lid = int(u[i])
        print(f'  {lid:>3} {ID2LABEL.get(lid, "?"):<16} {c[i] / node.latest.size:6.1%}')


if __name__ == '__main__':
    main()
