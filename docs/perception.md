# 障碍物感知模块（fp_ob_perception / fp_ob_ground / fp_ob_msgs）

方案依据：`ROS2感知搭建方案.md`（v2）。当前进度：**Phase 1 完成**（general_obstacle 几何分支，factory1 仿真 11/11 召回 0 虚警）。
与语义融合链路（`docs/fusion.md` 的 segformer 管线）并行存在，Phase 3 起合流。

## 0. 环境准备（同建图/导航）

```bash
source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
export PATH=/usr/bin:$PATH
```
感知推理节点（detect/preprocess/eval）跑在 conda `robot_ros2`（有 cv2/scipy/torch）：
```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate robot_ros2
```
> 注意：非交互 shell 里 `conda activate` 前必须先 source `conda.sh`；
> 手动 `ros2 run` 单节点时**必须带 `-p use_sim_time:=true`**（否则墙钟拒收 sim 时间戳 TF，节点静默无输出）；
> **conda 终端严禁 `export PATH=/usr/bin:$PATH`**——会把 conda activate 顶掉，节点用系统 python 崩在 import torch。该 export 只用于不开 conda 的终端（sim/ekf/scan/navigation，为避开 pytorch_yolo 3.11 的 rclpy 不兼容）。

## 1. 启动

```bash
# 终端1~3：仿真栈（见 build_map.md）
ros2 launch fp_factory_sim sim.launch.py world:=$PWD/src/fp_factory_sim/worlds/factory1.world \
    xacro:=$PWD/src/fp_factory_sim/urdf/factory_robot1.xacro
ros2 launch fp_factory_sim ekf.launch.py
ros2 launch fp_factory_sim scan.launch.py

# 终端4：感知栈（zband | patchwork 二选一）
ros2 launch fp_ob_perception obstacle_phase1.launch.py rviz:=true
ros2 launch fp_ob_perception obstacle_phase1.launch.py rviz:=true ground_mode:=patchwork
```

## 2. 数据流

```
/lidar/points_raw (lidar_link, 16线仿真)
  ├─ 地面分割（两方案并行保留，接口一致）
  │   ├─ zband:  fp_ob_perception/pointcloud_preprocess（TF→base_link，z带+距离过滤）
  │   │            -> /fp_ob/points_non_ground          ✅ 默认，11/11 0FP
  │   └─ patchwork: fp_ob_ground/patchwork_ground_node（C++，vendor Patchwork++ v1.4.1）
  │            -> /fp_ob/points_non_ground_pw           8/12 1FP（低装16线不适配，见§5）
  └─ obstacle_detect（输入话题可选，输出统一）
       聚类(eps 0.7) → OBB(minAreaRect) → 12棱线框+实心长方体
       odom系跟踪(AABB+质心混合关联, coasting 10帧) → /fp_ob/obstacles (ObstacleArray)
                                                    → /fp_ob/markers  (rviz)
```

消息（`fp_ob_msgs/Obstacle`）：`polygon`(OBB 4顶点, **odom系**) + `min/max_height`(点云真实高度) +
`class_name/confidence/id/source`。

## 3. 评估

```bash
# 机器人放回出生点(0,0)再测，否则 GT 区对比失真（血泪教训：挪车后左右货架"不对称漏检"是假象）
ros2 run fp_ob_perception eval_obstacles --duration 18
# 输出：12 个真值区（8货架+office/utility/machine/pallet）召回 + 虚警列表
```
实测基线（zband，16 线，原点）：**11/12 区检出、0 虚警**（office 与 utility/machine 邻接合并簇待 Phase 2 规整；墙沿并入货架属问题1，待地图过滤）。新增过滤：①薄板条 hspan<0.12 且(面积>1.2 或最长边>1.5)——清除低仰角掠射弧带（r5.9~7m、离地12~22cm 的 -3°/-5° 线伪影）；②min_points 6 + min_obb_area 0.06——清除孤立红点。

## 4. 关键参数（config/obstacle_params.yaml，全部实测标定）

| 参数 | 值 | 为什么 |
|---|---|---|
| `ground_band` | 0.13 | 通道内 ~12cm 高薄板杂点（-5°/-7° 线掠射带）0.12 时成弧带+桥接巨簇，0.13 归入地面；pallet 顶 +0.075 仍有 2cm 余量 |
| `min_range` | 2.0 | 车体近场自反射/轮区噪声；安全由 footprint+inflation 兜底 |
| `cluster_eps` | 0.70 | 16线 1° 角分辨率在 10m+ 掠射货架长面切向点距 ~0.3m，0.15/0.45 碎簇→闪烁 |
| `max_obb_extent` | 10.0 | 墙在角部连通成 31m 巨簇；墙体归 SLAM 静态层，感知只管物体级 |
| `track_match_dist` | 2.0 | 大货架视角变化质心跳动，配合 AABB 重叠混合关联 |
| `max_miss_frames` | 10 | coasting ~1.3s：障碍快进慢出，框不闪灭（半透明=coasting 中） |

## 5. 地面分割 A/B 结论（2026-10-21，factory1 原点公平对比）

| 方案 | 3线(旧) | 16线(现, factory_robot1 已合入) | 结论 |
|---|---|---|---|
| zband | 11/11, 0FP | **12/12, 1FP** | **默认**。低装场景实测最优 |
| Patchwork++（th 0.05/TGR off/RVPF on） | 8/12, 1FP | 11/12, **8FP**（通道地面波纹误判） | 保留为选项。低装+少线时 CZM/GLE 地面模型不稳：3线外圈无地面样本→漏；16线阈值漂移→虚警。≥32 线或 ≥1m 装高平台再启用 |

调参记录：TGR 自适应 `sensor_height` 室内漂移；RVPF 关闭更差(5/12)；`adaptive_seed_selection_margin` 随高度缩放无需改。

## 5b. 地图静态过滤（已上线，问题①关闭）

- 原理：障碍簇点（odom 系）→ TF → map 系，逐点查 `/map` 占据格（±0.25m 容差）；占据占比 > `static_frac_thresh`(0.6) 判静态结构丢弃
- 关键教训：必须用**原始簇点**采样——多边形边缘一半落在物体内部（地图里是 free），占比永远差一步
- 实测：开过滤 0 框（墙/货架/truck 全滤掉）；投放 0.5m 小箱到 (0,4) → 唯一存活框 (0.3,3.8) ✓
- 检测下限提示：0.4m 高小箱在 7m 处只有 2-3 个点（2° 垂直分辨率），过 ~5m 小目标不可见属物理极限
- 参数：`static_map_filter`（启动时读取，改后需重启 detect）、`static_frac_thresh`、`static_occ_level`
- ⚠️ 全终端必须 `export FASTDDS_BUILTIN_TRANSPORT=UDPv4`（已写入 ~/.bashrc）——WSL2 上 FastDDS 共享内存损坏会丢服务响应/TF，症状：lifecycle "Transitioning failed"、amcl 激活超时、节点收不到 /map

## 6. 已知问题 / 下一步

1. 碎片化：大平面切多条 → 多边形合并（Phase 2 计划内）
2. 薄板框（h 跨度 <0.08 的地面残留）→ 加高度跨度过滤
3. 移动中 id 仍有少量换号（EKF 抖动+视角变化）→ 跟踪加恒速模型
4. Phase 2：`raised_plastic_bag` 几何规则分支（低凸起+法向杂乱）
5. Phase 3：`vertical_glass` 地脚线法 + 净化分支（与 segformer 语义合流，见方案 §9）
6. Phase 4：`/fp_ob/obstacles` → costmap polygon layer，与 `/fp/traversability` A/B

## 7. 排障速查

| 现象 | 原因 |
|---|---|
| detect 零输出无报错 | use_sim_time 没开（手动 ros2 run 时）；或 TF 断（sim/ekf 没起） |
| 启动后前几秒 tf fail | DDS 发现期，正常，自动恢复 |
| 框横穿场景的斜线 | 线框顶点序 bug 已修；若复现查 marker ns/id 冲突 |
| 评估大面积漏检 | 先 `ros2 topic echo /odom --once` 确认机器人还在出生点 |
| patchwork 刷 "RNR requires intensity" | RNR 需强度列；已默认关（`enable_RNR:=false`）。真机 Hesai 带 intensity 时可开 |
| 远处大平面碎条/消失 | cluster_eps 与角分辨率几何关系，见 §4 |
