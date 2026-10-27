# fp_semantic_nav —— 玻璃语义绕行实验包（独立，不改现有代码）

链路：标定 → 仿真 → 感知(几何) → 语义融合(本包) → 导航绕行。

## 分阶段实验步骤

**阶段1 标定**（已就绪）：仿真用真值 `config/sim_truth_robot1.yaml`（K/R/t，
lidar(0.25,0,0.45)/cam(0.30,0,0.65) 俯视10°）。真机换 est.yaml 即可，接口不变。

**阶段2 仿真+感知底座**：
```bash
./script/fp_perception.sh sem        # sim+ekf+scan+几何感知+segformer 语义
```
rviz 确认：Seg Vis 里玻璃面板应为品红（windowpane/glass→transparent 组）。

**阶段3 玻璃检测（本包核心，单独可测）**：
```bash
# 新终端，source 环境后：
ros2 launch fp_semantic_nav detector.launch.py rviz:=true
# 在 Gazebo 里把玻璃摆到路中间：
ros2 run gazebo_ros spawn_entity.py -entity glass1 -x 2 -y 6 -z 0 -Y 1.57 \
    -file src/fp_factory_sim/models/window/model_lidar_passthru.sdf
```
验证：`/fp_sem/glass_cloud` 有点、rviz 里品红 `vertical_glass` 文字+青色地脚线
落在玻璃底边；`ros2 topic echo /fp_sem/glass_obstacles` 有 1 个多边形。

**阶段4 导航绕行**（替换原 navigation 终端）：
```bash
ros2 launch fp_semantic_nav glass_nav.launch.py rviz:=true
# 发 goal：目标在玻璃另一侧、直线必穿玻璃的位置，例如 (5,6)->(5,10) 之间放玻璃后
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 10.0}, orientation: {w: 1.0}}}}"
```
验证：Global Costmap 里玻璃位置出现紫色代价带，`/plan` 绕过玻璃，机器人实际绕行。

## 检测优先级（激光为主，语义补盲）——v2 设计
1. **negative（主）**：语义=实体类(wall/ceiling) 的像素区，其底沿比该列实体激光回波的最低投影行还低 ≥20px
   → "相机说是墙、激光说那里没有东西" = 透明面板，墙像素底边即地脚线。config/glass_params.yaml: wall_ids=[0,5]
2. **glass（辅）**：语义直接命中 windowpane/glass 类
3. **roi（兜底）**：/fp_sem/manual_roi 手动框（未来可换 YOLO 输出）
实测（蓝色合成玻璃@6m，无 YOLO 无手动）：自动命中 9630 点 → costmap 99 lethal → 导航绕行成功。
已知偏差：ceiling blob 底沿含少量地板像素 → 检出带在 3.2m（偏保守，实际玻璃 6m），待精调（候选列取中位/限制 blob 底沿连续性）。

## 原理与限制
- 地脚线法：mask 每列最底像素 → K/R 反投影到地面 → 玻璃底边线 → 拉帘点云
- costmap 集成：`nav2_glass_params.yaml` 在 local/global 障碍层加了第三观测源
  `glass`（marking-only，clearing=false）
- 限制：marking-only → 玻璃被"看见"后代价残留（静态玻璃无碍；挪走玻璃需重启导航或
  后续加 clearing 源）；玻璃需 segformer 能分对类（仿真 SkyBlue 半透明板通常判
  windowpane；识别不稳时调 detector 参数 glass_ids/min_area_px）
