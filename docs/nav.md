# 导航（Nav2）操作文档

fp_factory_sim + 已保存地图（maps/factory_map.yaml）的 Nav2 导航全流程。

## 1. 启动（三个终端）

```bash
source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
export PATH=/usr/bin:$PATH

# 终端1  ros2 launch fp_factory_sim sim.launch.py
# 终端2  ros2 launch fp_factory_sim ekf.launch.py
# 终端3  ros2 launch fp_factory_sim scan.launch.py
# 终端4  ros2 launch fp_factory_sim navigation.launch.py rviz:=true map:=$HOME/project/fusion_perception/src/fp_factory_sim/maps/factory1_map.yaml
```

**SLAM 与 Nav2 互斥**（都发布 map→odom），切导航前先杀掉 slam。

## 2. 初始位姿

- 机器人默认 spawn 在 (0,0) 朝 +x：nav2_params.yaml 已配 `set_initial_pose: True` +
  `initial_pose_x/y/a: 0,0,0`，**amcl 自动收敛，无需手动**
- 改了 spawn 位姿或定位跑飞时，手动给（rviz 工具栏 "2D Pose Estimate" 点+拖出朝向）：
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
- ⚠️ 本场景是 30×30 方形对称场地：**yaw 差 90° 会让 amcl 锁进完全等效的错误位姿**。
  给位姿时朝向必须大致正确（±30° 内）。
- 定位质量自检：`tf2_echo map base_link` 与 `tf2_echo odom base_link` 应基本重合（<10cm）。

## 3. 发送目标

rviz 工具栏 "Nav2 Goal"，或：
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 5.0}, orientation: {w: 1.0}}}}"
# 期望输出: Goal finished with status: SUCCEEDED
```
取消：`ros2 action send_goal` 界面 Ctrl+C 或 `ros2 service call /navigate_through_poses_cancel ...`/rviz 的 Nav2 面板。

## 4. 避障架构（当前已配置）

```
global_costmap = static_layer(factory_map) + ObstacleLayer(/scan 标记+清除, /scan_low 只标记) + inflation(0.6)
local_costmap  = 同上（voxel）
planner=DWB/NavFn, controller=DWB, cmd_vel 直连差速插件
```
- `/scan`（雷达平面 ±0.05m）：墙/机器等结构
- `/scan_low`（绝对高度 0.05~0.2m，-15° 线）：**托盘/线槽/下层箱等低矮盲区**，只加不减
- 实测：goal 直线穿托盘 (5,0.5) 时路径自动绕行

## 5. 验收用例

| 用例 | 操作 | 期望 |
|---|---|---|
| 直行 | goal (0,5) | SUCCEEDED，误差<0.3m |
| 低矮避障 | goal (8,0.5) 直线穿托盘 | 路径绕开，不卡死 |
| 动态避障 | 行驶中 rviz 再发新 goal | 重规划，无碰撞 |
| 卡死恢复 | 手动把车推到墙边（teleop） | BT 触发旋转/后退恢复 |

## 6. 常见问题

| 现象 | 原因/解决 |
|---|---|
| costmap 报 "frame odom/map does not exist" | 仿真没跑/残留进程打架：清场重启（见 build_map.md §0） |
| lifecycle 激活失败（Failed to change state） | 启动竞态，重启 navigation.launch.py 一次即好 |
| amcl 报 "cannot publish a pose" | 缺 `set_initial_pose: True`（已修）；或手动发 initialpose |
| 发 goal 不动 | `ros2 topic info /cmd_vel -v` 确认 diff_drive 在订阅；lifecycle 全 active |
| 定位跳变/镜像位姿 | 对称场地+初始 yaw 给错，重发正确 initialpose |
| 转弯欠 9% | 4 轮 caster 物理特性，SLAM/重规划可吸收，无需处理 |
| 导航中定位缓慢漂移 | amcl 已配 recovery_alpha_slow/fast=0.001/0.05（似然下降自动撒粒子自愈）+ update_min 0.1 + alpha 0.35；仍漂→查 EKF 是否在跑、地图是否有重影、货架周期混叠（跳 3m 整数倍→右侧货架 y 错开 1.5m 破对称） |

## 7. 下一步：语义融合导航（方案已评审通过）

- 阶段一：`fusion.launch.py` 输出 `/fp/filtered_points` 作为 costmap 第三观测源（二值避障）
- 阶段二：`/fp/traversability` 四级代价经 costmap Layer 插件叠加（软/硬/透明分级）
- 标定对接见 calib.md；执行细节待实施后补充本文档。
