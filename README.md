# fusion_perception

ROS2 Humble + Gazebo Classic 工厂场景下的 **激光×语义融合感知与导航** 完整工程：
从传感器标定、SLAM 建图、Nav2 导航，到几何障碍物检测、语义分割合流，
直至 **GlassGuard 范式的透明/镜面障碍（玻璃、积水、楼梯口）感知与绕行**。

```
标定(fp_calibration) → 仿真/真机传感 → 几何障碍(fp_ob_*) → 语义分割(fp_perception)
        → 表面提案+几何验证(fp_glassguard / fp_surface_guard) → costmap覆盖层(SurfaceOverrideLayer) → Nav2绕行
```

## 包结构

| 包 | 作用 |
|---|---|
| `fp_factory_sim` | 差速机器人 xacro、factory1 世界、SLAM/Nav2/EKF/scan 切片 launch、地图、玻璃模型(`models/`) |
| `fp_calibration` | 相机内参 + 相机-激光外参标定 |
| `fp_common` | 标定 IO、投影工具(project_points)、语义策略(policy.yaml) |
| `fp_perception` | SegFormer 语义节点、点云标注、Overlay、traversability 实验 |
| `fp_ob_msgs / fp_ob_perception / fp_ob_ground` | 几何障碍物管线(OBB+跟踪+静态图过滤)、Patchwork++ 地面分割 |
| `fp_semantic_nav` | 玻璃检测实验(v1/v2 detector、YOLO-ROI) |
| `fp_surface_guard` | **语义覆盖层**：C++ costmap2d Layer(覆盖 free 权限、分级代价、TTL) + 表面构建器 + `guard_check.py` 验收 |
| `fp_glassguard` | **GlassGuard 四阶段流水线**：柱状候选→投影验证→可修订假设管理→floor eviction |

## 快速开始（新机器）

```bash
git clone https://github.com/FakeItUtillMakeIt/fusion_perception.git
cd fusion_perception
bash migration/setup.sh        # ROS2+Gazebo+conda环境+torch(按GPU)+colcon build, 幂等
```
详见 [docs/10_migration.md](docs/10_migration.md)。已有环境则：
```bash
conda activate robot_ros2 && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install
```

## 演示：玻璃感知与绕行

```bash
# T1 底座(仿真+EKF+切片+segformer+RViz, 内部已处理 conda/UDPv4)
./script/fp_perception.sh sem
# T2 放置玻璃 (conda 终端)
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity glass1 -x 6 -y 0 -z 0 -Y 1.5708 \
  -file src/fp_factory_sim/models/window/model.sdf
# T3 GlassGuard 流水线 + 导航(含覆盖层)
ros2 launch fp_glassguard glassguard.launch.py
# T4 掩码提案(模拟微调模型输出; 真机换 YOLO/segformer 实例掩码即全自动)
ros2 topic pub -r 5 /fp_sem/manual_roi geometry_msgs/msg/PolygonStamped \
  "{polygon: {points: [{x: 90, y: 60}, {x: 550, y: 60}, {x: 550, y: 166}, {x: 90, y: 166}]}}"
# T5 发"穿玻璃"目标 → 机器人从面板边缘绕行
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 9.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
RViz 勾选 `/global_costmap/costmap`（玻璃处红带、激光点云穿过它）与
MarkerArray `/fp_glassguard/markers`（3D 假设线框+支持度）。

## 验证步骤

**一键验收**（逐级确认 仿真→标定→语义→提案→覆盖 全链路）：
```bash
bash migration/verify.sh                     # V1-V5 全 [OK] 即环境+链路正常
```

**语义生效四级验证**（决策链路验收）：
```bash
python3 src/fp_surface_guard/scripts/guard_check.py            # L1 labels→L2 surfaces→L3 costmap覆盖
python3 src/fp_surface_guard/scripts/guard_check.py --toggle   # L4 因果性: 撤提案→TTL后格子回落
```
| 级别 | 判据 | 断了查什么 |
|---|---|---|
| L1 语义在线 | `/fp/labels` ≈5Hz | segformer 崩→终端未 conda activate（`which python3`） |
| L2 表面提案 | `/fp_guard/surfaces` 非空 | 机器人是否正对玻璃 3-6m；ROI 是否在续发 |
| L3 覆盖生效 | 表面处 cost≥90、对照处 free | glassguard.launch 是否起了含插件的 nav2 参数 |
| L4 因果性 | 清空提案后同格回落 | Layer TTL(10s) |

**导航行为**：目标在玻璃对面 → 绕行 SUCCEEDED；停 ROI 15s 后带子消失 → 直通。

## 文档索引（docs/）

| 文档 | 内容 |
|---|---|
| [01_design_v1](docs/01_design_v1.md) | V1 设计：核心定位/功能/思路（历史） |
| [02_plan_v1_detail](docs/02_plan_v1_detail.md) | V1 详细实施方案+架构图（历史） |
| [03_plan_v2_pipeline](docs/03_plan_v2_pipeline.md) | **现行总体方案 v2**（标定→仿真→感知→融合→导航） |
| [04_calibration](docs/04_calibration.md) | 相机内参/外参标定操作 |
| [05_slam_mapping](docs/05_slam_mapping.md) | SLAM 建图操作 |
| [06_navigation](docs/06_navigation.md) | Nav2 导航操作 |
| [07_obstacle_perception](docs/07_obstacle_perception.md) | 几何障碍物感知模块 |
| [08_semantic_fusion](docs/08_semantic_fusion.md) | 语义×激光合流 |
| [09_fusion_map_status](docs/09_fusion_map_status.md) | 融合×建图导航 状态与操作总表 |
| [10_migration](docs/10_migration.md) | 迁移复现+已知坑表 |
| 包内 README | `fp_surface_guard`（覆盖层设计）、`fp_glassguard`（四阶段+调参教训）、`fp_semantic_nav`（v1/v2 实验） |

## 环境要求与关键约束

- Ubuntu 22.04（原生或 WSL2）+ ROS2 Humble + Gazebo Classic 11 + miniforge conda 环境 `robot_ros2`
- **WSL2 必须** `export FASTDDS_BUILTIN_TRANSPORT=UDPv4`（脚本已内置；手动终端需自行 export）
- **先 source ROS，再 conda activate，中途不要 `export PATH=/usr/bin:$PATH`**（会遮蔽 conda python，segformer 因缺 torch 秒死 → SegVis/Overlay "No Image"）
- torch/lxml 等只在 conda 环境里：spawn_entity、segformer、glassguard 都要在 `robot_ros2` 环境下运行
- 仿真前端限制：合成场景 segformer 分类摆动是已知问题，演示用 manual_roi 作为可信掩码源；真机需微调模型（路线见 docs/10_migration.md §5 与包 README）
