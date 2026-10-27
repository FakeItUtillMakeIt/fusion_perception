#!/usr/bin/env python3
"""语义->导航生效链路检查：四级逐级验证，一条命令给出结论。

  L1 语义在线      : /fp/labels 有数据
  L2 表面提案      : /fp_guard/surfaces 有非空 glass/water/stair 多边形(机器人需面对目标)
  L3 覆盖生效      : 表面多边形所在格子在 costmap 上是高代价, 而激光基线本该是 free
  L4 因果性(可选 --toggle): 清空 surfaces -> TTL 后格子回落 => 证明代价确实来自语义层

用法(conda 终端或任意能跑 rclpy 的地方):
    python3 src/fp_surface_guard/scripts/guard_check.py            # L1-L3
    python3 src/fp_surface_guard/scripts/guard_check.py --toggle   # 追加 L4(会短暂移除覆盖!)
"""
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from fp_surface_guard.msg import SurfaceArray


class Checker(Node):
    def __init__(self):
        super().__init__('guard_check',
                        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.lab_n = 0
        self.sur = None
        self.grid = None
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(Image, '/fp/labels', lambda m: setattr(self, 'lab_n', self.lab_n + 1),
                                 qos_profile_sensor_data)
        self.create_subscription(SurfaceArray, '/fp_guard/surfaces', self.sur_cb, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'grid', m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(SurfaceArray, '/fp_guard/surfaces', 10)

    def pose_cb(self, m):
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def amcl_cb(self, m):
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def sur_cb(self, msg):
        if msg.surfaces:
            self.sur = msg

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cell_cost(self, x, y):
        m = self.grid
        H, W = m.info.height, m.info.width
        g = np.array(m.data, dtype=np.int16).reshape(H, W)
        ix, iy = int((x - m.info.origin.position.x) / m.info.resolution), \
                 int((y - m.info.origin.position.y) / m.info.resolution)
        if not (0 <= ix < W and 0 <= iy < H):
            return -1
        return int(g[iy, ix])

    def map_ring(self, s):
        # 表面多边形 -> map 系质心+最小距离点(简化: 取第一个点的变换近似, 用 odom~map 差)
        return np.array([(p.x, p.y) for p in s.polygon.points])


def main():
    toggle = '--toggle' in sys.argv
    rclpy.init()
    ck = Checker()
    print('== L1 语义在线 (/fp/labels) ...')
    ck.spin_for(6)
    hz = ck.lab_n / 6.0
    print('   /fp/labels %.1f Hz -> %s' % (hz, 'OK' if hz > 0.5 else 'FAIL(segformer 没起/没 conda)'))
    if hz <= 0.5:
        print('   修复: 终端里 conda activate robot_ros2 && which python3 应指向 robot_ros2 再重启栈')
        return

    print('== L2 表面提案 (/fp_guard/surfaces) ...')
    ck.spin_for(12)
    if ck.sur is None:
        print('   FAIL: 无非空表面。检查 surface_builder 是否在跑、机器人是否面对玻璃、')
        print('         合成场景类别是否在 config/surface_guard.yaml 的 ids 里(sim: 130/0/5)')
        return
    s = ck.sur.surfaces[0]
    ring = ck.map_ring(s)
    print('   OK: %d 个表面, 首个 class=%s conf=%.2f (base系 x[%.1f,%.1f])'
          % (len(ck.sur.surfaces), s.class_name, s.confidence, ring[:, 0].min(), ring[:, 0].max()))

    print('== L3 覆盖生效 (costmap) ...')
    ck.spin_for(4)
    if ck.grid is None or ck.pose is None:
        print('   FAIL: 缺 /global_costmap/costmap 或 /amcl_pose(未初始化)')
        return
    sx = np.mean(ring[:, 0]); sy = np.mean(ring[:, 1])
    rx, ry, wy = ck.pose
    c_, s_ = np.cos(wy), np.sin(wy)
    gx, gy = rx + c_ * sx - s_ * sy, ry + s_ * sx + c_ * sy
    cost = ck.cell_cost(gx, gy)
    side = ck.cell_cost(rx + c_ * (sx + 1.5) - s_ * sy, ry + s_ * (sx + 1.5) + c_ * sy)
    print('   表面质心 map(%.1f,%.1f) cost=%d ; 旁移1.5m对照 cost=%d' % (gx, gy, cost, side))
    ok3 = cost >= 90
    print('   %s: 表面处=%s 对照处=%s' % ('OK(覆盖写入致命带)' if ok3 else '检查TTL/frame_id',
          'lethal' if ok3 else '非致命', 'free' if 0 <= side < 50 else '非free'))
    if toggle:
        print('== L4 因果性: 连续 15s 发空快照 -> TTL 到期 -> 覆盖应撤除 ...')
        t0 = time.time()
        while time.time() - t0 < 15:
            m = SurfaceArray()
            m.header.stamp = ck.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            ck.pub.publish(m)
            ck.spin_for(0.3)
        ck.spin_for(3)
        cost2 = ck.cell_cost(gx, gy)
        print('   撤除后同点 cost: %d -> %d -> %s' % (
            cost, cost2, 'OK(覆盖确实来自 guard 层)' if cost2 < 90 else '未回落(检查builder是否在补投)'))
        print('   注: builder 下一帧会重新提案, 撤除窗口只有 TTL(默认10s)量级, 之后点云带恢复')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
