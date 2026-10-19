## 3D LiDAR + RGB/RGB-D 可通行路径融合感知详细实施方案

### 系统架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                     传感器层                                 │
│  3D LiDAR ──┐                                              │
│  RGB 相机 ──┼── PTP/硬触发时间同步 ──→ 统一时间戳            │
│  RGB-D 相机─┘                                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                   标定与对齐层                                │
│  离线标定（棋盘格/RANSAC）→ 在线精化（语义特征匹配）           │
│  LiDAR→Camera投影矩阵 K·[R|t]                               │
│  Camera→LiDAR反投影 + 深度补全                               │
└──────────────────────┬──────────────────────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        │                             │
┌───────▼────────┐          ┌─────────▼──────────┐
│  RGB 分支      │          │  LiDAR 分支         │
│  语义分割      │          │  地面分割           │
│  透明/玻璃检测 │          │  几何特征提取       │
│  深度不一致分析│          │  负障碍物检测       │
│  BEV投影       │          │  BEV投影            │
└───────┬────────┘          └─────────┬──────────┘
        │                             │
┌───────▼─────────────────────────────▼──────────┐
│              BEV 特征融合层                      │
│  语义通道 + 几何通道 + 透明否决通道 + 置信度通道  │
│  贝叶斯决策 / 注意力融合 / 安全否决规则           │
└──────────────────────┬─────────────────────────┘
                       │
┌──────────────────────▼─────────────────────────┐
│         可通行性代价地图输出                     │
│  四级可通行性标签 + 连续代价值                   │
│  直接接入 Nav2 / TEB / MPPI 规划器              │
└────────────────────────────────────────────────┘

```

RGB 分支：语义分割采用segformer b2

---

## 当前进度

- [x] 阶段一 MVP：LiDAR 点云语义着色管线（分割→投影→打标→策略着色→filtered 输出）
- [x] 仿真环境（假 RGB + 16 线 LiDAR + 标定真值），全流程闭环验证
- [x] 内参标定工具（浏览器 UI，仿真验证 K 误差 <0.5%）
- [x] 外参标定：板级自动解算 board_extrinsics（仿真验收 旋转 0.47° / 平移 3.9mm / 重投影 4px）
- [x] 真实照片分割预检工具 image_publisher（person/wall/curtain 稳定输出）
- [x] 真实视频预检：公路行车视频 sem_vis 稳定，label_probe 确认路面 100% road；
      高光地面帧间抖动 → segformer 加 smooth_k logits 时域平滑（默认 3）
- [x] **可通行性栅格节点 traversability_grid**：语义证据衰减累积 → OccupancyGrid 四级代价
      （0/50/65/90/98），仿真验证墙线正确落在 x=4m、10Hz 输出
- [x] **Gazebo 工厂仿真 + SLAM + Nav2 闭环**（fp_factory_sim 包）：差速底盘+3D LiDAR+相机+IMU，
      factory.world(30×30+4 障碍)，pointcloud_to_laserscan→slam_toolbox 建图（墙单线无鬼影），
      map_saver→nav2(map_server+amcl+DWB) 导航 SUCCEEDED；详见 todo.md 十二/十三节
- [ ] 真实相机 + Hesai 驱动接入，真机标定 + 同步录 bag 回放验证
- [ ] policy.yaml 按实际失败场景调优，/fp/filtered_points 接 costmap A/B
- [ ] 二期：透明/玻璃否决通道（LiDAR 无回波 + 图像语义交叉验证）、RGB-D 近场补盲

## 阶段一实现：LiDAR 点云语义着色 MVP

运行环境：x86 + NVIDIA GPU（RTX 5060 实测），ROS 2 Humble，conda env `robot_ros2`（numpy 固定 1.26.4 兼容 cv_bridge；torch cu130 + transformers 5.x）。

### 包结构
```
src/fp_common       标定/投影/策略/ADE20K 标签库 + webview(浏览器 UI 框架)
src/fp_calibration  camera_intrinsics(内参) manual_align(手动外参) board_extrinsics(板级自动外参)
src/fp_perception   segformer_node(ADE20K 分割) lidar_label_node(逐点打标+策略过滤)
src/fp_bringup      fusion.launch + config/{extrinsics,policy}.yaml + config/fp_debug.rviz
src/fp_sim          fake_sensor(仿真传感器, publish_image:=false 可只发点云) image_publisher(照片/视频预检)
                    eval_extrinsics(标定验收) label_probe(区域类别探测)
```

### 数据流
```
/camera/image_raw → segformer_node → /fp/labels (mono8) + /fp/sem_vis
/hesai/points  ──┐
/fp/labels      ─┴→ lidar_label_node → /fp/labeled_points (x,y,z,label)
                                    → /fp/points_policy  (x,y,z,rgb 按策略着色)
                                    → /fp/filtered_points (按 costmap_drop_groups 剔除后可直接替换现有 costmap 输入)
/fp/labeled_points → traversability_grid(TF到odom, 语义证据时间衰减累积)
                 → /fp/traversability (nav_msgs/OccupancyGrid, 0~100 代价, -1 未知)
                 → /fp/traversability_vis (俯视调试图)
/camera/image_raw + /fp/labeled_points → overlay_node → /fp/overlay (语义代价投影叠加实景, 高斯羽化)
```

### 画面叠加 overlay_node
把带标签点云投影回相机画面并按策略组半透明着色（绿可通行/黄软/红硬/品红透明），
直观检查标定精度与语义正确性。参数：`alpha`(透明度 0.45)、`splat_px`(羽化半径 9)、
`stride`(点抽稀 2)。fusion.launch 已自动启动；rviz 预置含 Overlay 画面。
真机上此画面就是最实用的标定验收工具：彩色投影应严丝合缝贴在物体表面上。

### 可通行性栅格 traversability_grid
参数（fusion.launch 已自动接入，需 TF: odom→lidar_link）：
| 参数 | 默认 | 说明 |
|---|---|---|
| resolution / extent | 0.05 / 10.0 | 栅格分辨率(m)、边长(m)，机器人居中滚动 |
| decay_tau | 2.0 | 证据时间衰减常数(s)，越大记忆越长 |
| soft/hard/transparent_cost | 50/90/98 | 三组障碍代价（安全优先 transp>hard>soft）|
| unknown_cost | 65 | 相机视野外/未分类的几何点（保守可通行）|
| min_z / max_z | -0.7 / 2.5 | 相对传感器高度的点过滤 |

接 Nav2 两种方式：
1. **简单**：用 `/fp/filtered_points` 替换现有 costmap 的点云输入（二值障碍）
2. **完整**：`/fp/traversability` 作为独立代价源——写个 nav2_costmap_2d Layer 插件订阅它叠加到
   master costmap（可表达软/硬分级），或用 `costmap_filter`/`nav2_ctr` 直接以该栅格规划

## 标定工具链（全部浏览器 UI）
> WSLg 下 OpenCV 高gui 窗口任务栏点击不恢复，所有标定 GUI 已改为浏览器 UI
> （fp_common/webview.py：MJPEG 流 + 滑条 + 按钮 + 点击选点），Windows 浏览器开 localhost。

```bash
source /opt/ros/humble/setup.bash
conda activate robot_ros2
cd fusion_perception && colcon build --symlink-install && source install/setup.bash
# 0) 启动模拟标定板 
ros2 launch fp_sim fake_sensor.launch.py board_motion:=true

# 1) 相机内参: 浏览器 http://localhost:8090, [grab] 抓 15~25 帧(多姿态/多位置),
#    [quit] 退出并自动解算写出 yaml
ros2 run fp_calibration camera_intrinsics capture --topic /camera/image_raw --dir ./shots \
    --board 11x8 --square 0.025 --out camera_calib.yaml

# 2) 外参(推荐, 板级自动解算): 浏览器 http://localhost:8092
#    原理: 图像棋盘格->板在相机系位姿; LiDAR 板平面 PCA 矩形->板在雷达系位姿; 两者求外参
#    看到 "board FOUND" 按 [sample], 移动标定板换姿态采 15~20 个(跨 2~3 个摆动周期),
#    [finish] 写出, [quit] 退出。--init 为初始姿态猜测(hesai90=机械式 x前/y左/z上), 需与真值差 <45°
#    仿真实测: 旋转 0.47°, 平移 3.9mm, 全距离重投影差 ~4px
ros2 run fp_calibration board_extrinsics --calib camera_calib.yaml --out est.yaml \
    --board 11x8 --square 0.035

# 2b) 外参(备用, 无标定板时): 浏览器 http://localhost:8091
#    滑条粗对齐 -> [pick:on/off] 点>=8 个同名角点(直接点 LiDAR 圆点中心, 深度拉开)
#    -> [pnp] -> [save]。注意: 稀疏点吸附对光轴方向平移约束弱, dz 误差可达 10cm+
ros2 run fp_calibration manual_align --calib camera_calib.yaml --out est.yaml \
    --image-topic /camera/image_raw --lidar-topic /hesai/points

# 3) 标定验收(仿真有真值时): 旋转/平移误差 + 各距离重投影差
ros2 run fp_sim eval_extrinsics est.yaml sim_truth.yaml
```

### 运行融合
```bash
ros2 launch fp_bringup fusion.launch.py lidar_topic:=<点云topic> image_topic:=<图像topic> calib:=est.yaml
rviz2 -d $(ros2 pkg prefix fp_bringup)/share/fp_bringup/config/fp_debug.rviz
# 预置: FixedFrame=lidar_link, /fp/points_policy(RGB8着色), /fp/sem_vis, /camera/image_raw
# 真机省算力: -p every_n:=2 (分割降到 15Hz)
```
调优：`policy.yaml` 中 ADE20K 类别名/ID → passable/soft/hard/transparent 分组；
`costmap_drop_groups` 填组名即可在 filtered_points 中剔除对应点（先只观察、不剔除，A/B 稳定后再开）。

### 颜色语义（sem_vis 与 points_policy 一致，需带 policy_path 启动）
| 颜色 | 分组 | 含义 |
|---|---|---|
| 绿 | passable | road/sidewalk/floor/path… 可通行 |
| 黄 | soft | grass/plant/tree… 植被软障碍 |
| 红 | hard | person/car/fence/wall… 硬障碍 |
| 品红 | transparent | windowpane/glass/mirror… 待二期否决通道 |
| 灰 | unknown | 相机视野外 / 未分类 |

排查工具：`ros2 run fp_sim label_probe [--region x0,y0,x1,y1]`（默认取样画面下方中央=地面区，
打印该区域主导类别）。地面显示品红 = 被误判为 glass/mirror（高光地面常见），
用 label_probe 确认后调整 policy 分组或依赖二期 LiDAR 回波交叉验证。
帧间颜色闪烁 = 决策边界抖动，调 `smooth_k`（logits 时域平均帧数，默认 3，抖动大用 5~8，
代价是标签滞后 K/10 秒）。

## 仿真环境（无实物开发/练标定用）

`fp_sim/fake_sensor`：合成室内场景（透视地砖、墙、柜/箱/车/杆、棋盘格可摆动或 `board_motion:=false` 静止）
+ 16 线 LiDAR 光线追踪（话题名与实物一致），当前目录写 `sim_truth.yaml` 标定真值，发布 TF。

```bash
ros2 launch fp_sim fake_sensor.launch.py                      # 终端1
ros2 launch fp_bringup fusion.launch.py calib:=est.yaml       # 终端2 (或 sim_truth.yaml)
rviz2 -d $(ros2 pkg prefix fp_bringup)/share/fp_bringup/config/fp_debug.rviz   # 终端3
```

仿真实测：segformer 端到端 ~13fps（GPU 推理 18ms），点云打标覆盖率 ~50%（=相机 FOV 几何占比）。

### 真实照片/视频预检（上真机前验证分割质量）
```bash
ros2 run fp_sim image_publisher --path /path/to/工作场景照片.jpg        # 或 .mp4 视频(自动按原帧率循环)
ros2 run fp_perception segformer_node --ros-args -p policy_path:=src/fp_bringup/config/policy.yaml
# rviz 看 /fp/sem_vis: 真实图像输出稳定(实测公路视频: 路面 100% road=绿, 护栏红, 天空灰)
```
**关键预检**：用机器人实际工作环境的照片/视频（玻璃/植被/栏杆等纯 LiDAR 失败场景）跑 sem_vis +
label_probe，确认目标物体能被正确分割，再决定上真机。

### 视频 + 仿真点云混合（看三维打标效果，无真机时）
```bash
ros2 launch fp_sim fake_sensor.launch.py publish_image:=false     # 终端1: 只发仿真点云
ros2 run fp_sim image_publisher --path 视频.mp4                   # 终端2: 视频当图像源
ros2 launch fp_bringup fusion.launch.py calib:=est.yaml           # 终端3
rviz2 -d $(ros2 pkg prefix fp_bringup)/share/fp_bringup/config/fp_debug.rviz   # 终端4
```
注意：仿真点云(室内)与真实视频(室外)几何不对应，此组合只验证**打标机制**
（覆盖率/FOV 边界/颜色映射），真实对齐效果需真机同步录包：
`ros2 bag record /camera/image_raw /hesai/points` 回放 + 跑融合。

## 真机接入清单
1. `hesai_ros_driver`（官方 ROS2 驱动，P16/XT32）+ 相机驱动；确认点云 frame 为 x前/y左/z上（否则 `--init` 换猜测或改 R_PRESET）
2. 打印 11×8 内角点棋盘格（方格 ≥40mm，贴平硬板）
3. camera_intrinsics → board_extrinsics → rviz 检查 `/fp/points_policy` 边缘贴合度（真机验收标准）
4. 对照实际失败场景调 `policy.yaml`，A/B：`/fp/filtered_points` 替换现有 costmap 点云输入

## 坑位记录
- WSLg 窗口不渲染/任务栏点不出：`wsl --shutdown` 重启；OpenCV 高gui 彻底不可靠 → 全部改浏览器 UI
- 残留节点进程抢 GPU：launch 被 kill 后子进程可能不退出，多个 segformer 让推理 15ms→100ms+；用 `pkill -f '[l]ib/fp_'` 清理（注意 pkill 模式加中括号防止匹配到自己的 shell）
- numpy 固定 1.26.4（cv_bridge 是 numpy1 ABI）；opencv-python 对应降到 4.11
- ament_python 的 console_scripts/entry_points 装到 bin/ 而非 lib/<pkg>/（launch 找不到），本项目用 data_files + scripts/ wrapper
- 理想棋盘格对 LiDAR 点云存在 180° 旋转歧义（图案反对称），用"与初始猜测最接近"消歧，故 --init 必须差 <45°
- OpenCV 角点排序有 4 重对称性，依赖图案相位的判别必须每帧从图像锚定
- eval_extrinsics 测试点云注意 LiDAR 系是 x 前向（z 前向会让重投影误差虚高 100 倍）
- sem_vis 单独跑 segformer 时记得传 `-p policy_path:=...`，否则是随机类别配色（易误以为误判）
- WSLg 整体不渲染任何 X 窗口（rviz 空/无图标恢复）时：PowerShell `wsl --update; wsl --shutdown` 后重开
- 每 colcon build 出新包后，已开的终端必须重新 source install/setup.bash，否则 Package not found
- StaticTransformBroadcaster 的 transient_local 历史深度=1：多次 sendTransform 后到的订阅者只能收到
  最后一条！所有静态 TF 必须合并成一次 sendTransform([tf1, tf2])
- 仿真 odom 系按 ROS 惯例对齐雷达系(x前/y左/z上)，camera_link 挂光轴变换；栅格 z 过滤才正确
- OccupancyGrid data 行主序 [y*width+x]，别多此一举转置
- ApproximateTimeSynchronizer 队列按最高频话题算：30Hz 图像 queue=5 只有 0.17s 窗口，
  配不上延迟 ~0.4s 的标签话题 → overlay 等同步节点 queue 要给到 50、slo 0.15

