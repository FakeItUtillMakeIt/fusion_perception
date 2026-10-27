# 建图（SLAM）操作文档

fp_factory_sim（ROS2 Humble + Gazebo Classic）建图全流程。
适用：仿真雷达+轮里程计+IMU（EKF 融合）构建 2D 栅格地图，供 Nav2 导航使用。

## 0. 环境准备（每个终端都要执行）

```bash
source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
export PATH=/usr/bin:$PATH        # 关键：避开 conda python（spawn_entity/map_saver/ekf 等是 python 脚本）
```

**铁律**：同一时刻只能有一套仿真栈（fp_factory_sim 或 fp_gazebo）。启动前/切换前清场：
```bash
pkill -9 -f "ign gazebo|gzserver|gzclient|robot_state_pub|pointcloud_to_l|async_slam|ekf_node|rviz2|amcl|controller_server|bt_navigator|teleop"
```

## 1. 场景选择

| 场景 | world | xacro | 布局 |
|---|---|---|---|
| 基础（默认） | factory.world | factory_robot.xacro | 30×30 开阔场+4 障碍，练手用 |
| 货架厂房 | factory1.world | factory_robot1.xacro | 中央通道 x∈[-6,6]，货架行 x=±10（y=1/4/7/10），南侧墙有开口，office(-8,-10)/utility(8,-10)/machine(7.5,-6.5)/pallet(-6,-6.5)/truck(±10,12.5) |

## 2. 启动（四个终端，按顺序）

```bash
# 终端1：仿真（headless:=false 弹 Gazebo 3D 窗口）
# 基础场景：
ros2 launch fp_factory_sim sim.launch.py headless:=false
# 货架厂房场景：
ros2 launch fp_factory_sim sim.launch.py \
    world:=$HOME/project/fusion_perception/src/fp_factory_sim/worlds/factory1.world \
    xacro:=$HOME/project/fusion_perception/src/fp_factory_sim/urdf/factory_robot1.xacro \
    headless:=false

# 终端2：EKF 融合（轮 odom + IMU -> 发布 odom→base TF；diff_drive 插件已关 odom TF）
ros2 launch fp_factory_sim ekf.launch.py

# 终端3：点云->2D 切片（/scan 高位给 SLAM，/scan_low 低位给避障）
ros2 launch fp_factory_sim scan.launch.py

# 终端4：SLAM + rviz
ros2 launch fp_factory_sim slam.launch.py rviz:=true
```

快速自检（第 5 个终端）：
```bash
ros2 topic hz /scan                 # ≈10Hz
ros2 topic hz /odometry/filtered    # EKF 输出，>0 即正常
ros2 run tf2_ros tf2_echo odom base_link   # 连续变换（来源=ekf_filter_node）
```
rviz：Fixed Frame=map，激光红线应贴墙；地图要等机器人动起来才出现（minimum_travel_distance=0.15m）。

## 3. 驾驶建图

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- 速度：`z` 降线速到 0.2~0.3，`b` 角速 ≤0.5（高重心+4轮，快了晃、打滑、 odom 撒谎）
- 通用策略：先主通道再分支；匀速直行少画龙；端点停留扫特征；走完原路低速返回一次触发回环；收尾在开阔处原地转一圈

**基础场景禁区**（扫描面 0.8m 高，看不见但过不去）：线槽 x∈[5,7],y=-2（卡死过 3 次）、托盘 (5,0.5)、箱堆 (8,-1)。
路线：内圈 ±2m 方形 → 外圈 ±10m → 开阔处旋转收尾。

**货架场景要领**：
- **走中央通道**（x∈[-6,6]，宽 12m），**不要钻进货架间隙**（~1m 宽平行走廊 = 2D 退化+物理蹭架）
- 货架侧面从中央通道即可完整扫到；南北两端开口（y=±15 南侧 x∈[-3,3]、truck 区）多停留
- office/utility/machine/pallet 区（y<0）低速绕行，pallet(-6,-6.5) 是低矮盲区注意保持距离
- 纵向漂移对策见 §6

## 4. 保存地图（两种都存）

```bash
# A. Nav2 用（pgm+yaml；导航 launch 用 map:= 指向它）
ros2 run nav2_map_server map_saver_cli -f src/fp_factory_sim/maps/factory1_map --ros-args -p use_sim_time:=true

# B. slam_toolbox 续图用（posegraph）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: /home/sevnce/project/fusion_perception/src/fp_factory_sim/maps/factory1_posegraph}"
```
另有自动检查点：`save_period: 30` 每 30s 自动序列化 posegraph 到 **slam 终端的当前目录**（`map2026-*.posegraph`），漂移严重时可用它回滚重建。

## 5. 续图（基于已建地图继续）

```bash
ros2 launch fp_factory_sim slam.launch.py rviz:=true \
    map_file_name:=/home/sevnce/project/fusion_perception/src/fp_factory_sim/maps/factory1_posegraph
```
注意：map_saver 的 pgm/yaml 不能被 slam_toolbox 重载，续图必须用 posegraph。

## 6. 质量验收

看 pgm（`eog src/fp_factory_sim/maps/factory1_map.pgm`）：
- ✅ 四面墙单线无重影（重影=打滑/TF 断/走廊漂移）
- ✅ 货架行 x=±10 呈现规则梳齿状（长边 8m 侧面+端头）
- ✅ machine/office/utility/truck 实心块位置正确
- ✅ pallet 不出现（低矮盲区正常，导航层 scan_low+语义补盲）
- 实测参考（基础场景）：线宽 ≤20cm、零重影

## 7. 常见问题

| 现象 | 原因/解决 |
|---|---|
| 刷屏 "queue is full" | world 的 `real_time_factor` 必须=1（0=超实时洪峰） |
| 地图不动/无 TF | 残留进程打架——先跑 §0 清场命令 |
| 车走不动轮子空转 | caster 间隙 3mm（0 会打滑、>6mm 会点头）；轮子禁加 minDepth/damping |
| rviz 报缺轮子 TF | xacro 内 joint_state_publisher 插件已配，重启 sim |
| 货架走廊纵向漂移 | 平行面走廊=2D 退化；EKF 已缓解，驾驶策略（§3）必须配合：走中央通道、匀速直行、端点停留、原路往返 |
| 卡墙后 odom 乱跳 | 速度控制里程计在打滑时撒谎——EKF(odom+IMU) 已压制；真机化后调大 covariance_* |
| EKF 启动报 params file 错误 | ekf.yaml 的 process_noise_covariance 必须全为浮点（混 int 会解析失败） |
| 建图时 amcl/nav2 在跑 | 二者抢 map→odom TF——SLAM 与 Nav2 互斥，切换先杀进程 |

## 8. 建图完成后

导航启动方法见 `docs/06_navigation.md`（注意启动顺序同样要加 ekf.launch.py）。
