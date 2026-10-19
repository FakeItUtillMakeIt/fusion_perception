# 建图（SLAM）操作文档

fp_factory_sim（ROS2 Humble + Gazebo Classic）建图全流程。
适用：用仿真雷达+里程计构建 2D 栅格地图，供 Nav2 导航使用。

## 0. 前置：环境

```bash
source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
export PATH=/usr/bin:$PATH        # 关键：避开 conda python（spawn_entity/map_saver 是 python 脚本）
```

**铁律**：同一时刻只能有一套仿真栈（fp_factory_sim 或 fp_gazebo），切换前清场：
```bash
pkill -9 -f "ign gazebo|gzserver|robot_state_pub|pointcloud_to_l|async_slam|rviz2"
```

## 1. 启动（三个终端）

```bash
# 终端1：仿真（headless:=false 开 3D 窗口）
ros2 launch fp_factory_sim sim.launch.py

# 终端2：点云->2D 切片（同时发 /scan 高位 + /scan_low 低位）
ros2 launch fp_factory_sim scan.launch.py

# 终端3：SLAM + rviz
ros2 launch fp_factory_sim slam.launch.py rviz:=true
```

快速自检：
```bash
ros2 topic hz /scan          # ≈10Hz
ros2 run tf2_ros tf2_echo odom base_link   # 有连续变换
```
rviz 中 Fixed Frame=map；激光红线应贴在墙上。地图要等机器人动起来才出现（minimum_travel_distance=0.15m）。

## 2. 驾驶建图

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- 速度：`z` 降到 0.2~0.3，`b` 角速 0.5 以内（高重心+4轮，快了会晃、会打滑）
- 贴墙 1~3m 绕场；先内圈（±2m 方形，净空）再外圈（±10m），最后在开阔处原地转一圈补视角

**避障禁区**（2D 扫描面 0.8m 高，看不见但底盘过不去）：
| 物体 | 位置 | 说明 |
|---|---|---|
| 线槽 trench | x∈[5,7], y=-2.0 | 6cm 台阶，卡死过 3 次 |
| 托盘 pallet | (5.0, 0.5) | 15cm 高 |
| 箱堆 stacks | (8.0, -1.0) | 下层 60cm 部分可见 |

## 3. 保存地图（两种方式都存）

```bash
# A. Nav2 用（.pgm+.yaml，navigation.launch.py 默认读这个路径）
ros2 run nav2_map_server map_saver_cli -f src/fp_factory_sim/maps/factory_map \
    --ros-args -p use_sim_time:=true

# B. slam_toolbox 续图用（.posegraph+.data）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: /home/sevnce/project/fusion_perception/src/fp_factory_sim/maps/factory_posegraph}"
```

## 4. 续图（基于已建地图继续建）

```bash
ros2 launch fp_factory_sim slam.launch.py rviz:=true \
    map_file_name:=/home/sevnce/project/fusion_perception/src/fp_factory_sim/maps/factory_posegraph
```
注意：`map_saver` 的 pgm/yaml **不能**被 slam_toolbox 重新加载，续图必须用 posegraph。

## 5. 建图质量验收

看 `factory_map.pgm`（eog 打开）：
- ✅ 四面墙单线、无重影（重影=开太快打滑或 TF 断）
- ✅ machine (3.5,2.0) 实心块；箱堆上层有回波
- ✅ pallet/线槽**不出现**（低于扫描面，属正常 2D 盲区，由导航层 scan_low+语义补盲）
- 实测参考：线宽 ≤20cm、东墙覆盖 9/9、零重影

## 6. 常见问题

| 现象 | 原因/解决 |
|---|---|
| 刷屏 "queue is full" | world 的 real_time_factor 必须=1（0=超实时洪峰） |
| 地图不动/无 TF | 清残留进程（两套仿真互踩是头号原因） |
| 车走不动轮子空转 | caster 间隙 3mm（不能 0 不能 >6mm）；轮子禁加 minDepth/damping |
| 建图期 rviz 报缺轮子 TF | joint_state_publisher gazebo 插件已在 xacro 内，重启 sim 即可 |
