# fp_surface_guard — GlassGuard 范式：语义表面覆盖层

把"视觉提议候选面 + 激光种子几何验证 + 覆盖写入占据"落地为通用机制，
统一处理**激光谎报 free 的危险表面**：玻璃（透射/镜面）、积水（镜面）、下行楼梯口（越沿打远）。

## 与 fp_semantic_nav 旧方案的本质区别

| | glass_detector v2（旧） | fp_surface_guard（本包） |
|---|---|---|
| 存在性权威 | 激光（无回波是必要条件） | **视觉**（掩码直接成立，激光无否决权） |
| 激光角色 | 门神 | 种子点修正深度 / 补盲负候选 |
| 写入通道 | obstacle_layer 观测源（与 raytrace 清除共享话语权） | **专用 Layer 覆盖 free**（排在障碍层后、膨胀层前） |
| 横放地面玻璃 | 原理上测不了 | 掩码成立即可覆盖 |
| 类别策略 | 单一 | class -> cost/ttl 表（致命/高代价分级） |

## 结构

```
scripts/surface_builder.py      视觉提议 -> 候选面(多边形) -> 种子几何修正 -> /fp_guard/surfaces
  glass  : glass_ids 直检 | wall_ids 负候选(墙像素但激光无回波证据) | 手动ROI  -> 'glass' 竖直条带
  water  : water_ids 直检 -> 掩码地面包络凸多边形                            -> 'water' 贴地面
  stair  : stair_ids 直检 -> 同上                                            -> 'stair_down'
src/surface_override_layer.cpp  nav2_costmap_2d::Layer 插件
  订阅 SurfaceArray -> TF(最新)转 costmap 全局系 -> 多边形栅格化
  max 写入 class 代价(覆盖自由空间)；TTL 过期恢复原格；滚动窗安全(世界坐标记账)
msg/Surface2D.msg               header(frame=base_link) class confidence polygon z范围
```

代价刻度（nav2 内部值，发布 OccupancyGrid 时重映射 ~x/2.54）：
`glass/stair_down=254(致命)  water=150(高代价, planner 尽量绕、可强行穿越)`。

## 使用

```bash
./script/fp_perception.sh sem      # 底座: sim+ekf+scan+语义(脚本内部已 conda activate)
# spawn 玻璃（模型在 ~/model_editor_models, spawn 需在 conda 终端: 要 lxml）
ros2 run gazebo_ros spawn_entity.py -entity glass1 -x 6 -y 0 -z 0 -Y 1.5708 \
  -file src/fp_factory_sim/models/window/model.sdf
ros2 launch fp_surface_guard guard.launch.py     # builder + nav2(含覆盖层)
```

## 验证语义是否在导航里生效

```bash
python3 src/fp_surface_guard/scripts/guard_check.py            # L1 labels -> L2 surfaces -> L3 costmap
python3 src/fp_surface_guard/scripts/guard_check.py --toggle   # 追加 L4: 清空提案->TTL 后格子回落(因果性)
```
L3 判据：表面质心格子 >=90(致命) 且 1.5m 旁对照格 free —— 激光在该处本来报 free，
是语义层把它顶成了致命带。目视法：RViz 勾 /global_costmap/costmap，玻璃位置出现红带，
而 /scan 点云直接穿过去。

## 常见故障

- **SegVis/Overlay 'No Image'**：segformer 没起来（`ros2 topic hz /fp/labels` 为 0）。
  根因几乎都是终端环境：torch/lxml 只在 robot_ros2 里。新开终端直接
  `./script/fp_perception.sh sem`；若手动启动，先 `conda activate robot_ros2`
  且**不要**在 activate 之前 `export PATH=/usr/bin:$PATH`（会遮蔽 conda python）。
  labels 一断：seg_vis 无话题、overlay 无输入、builder 无提案、guard 层空转，
  表现就是"图像没了 + 语义好像没作用"。

调参：`config/surface_guard.yaml`（builder）；`config/nav2_guard_params.yaml`
（= 基础 nav2 + surface_guard 插件块，class_names/class_costs/class_ttls）。

## 防漂移设计（本轮实测教训）

1. **世界系锚定 + 滞回锁**：表面在 base_link 每帧重提案会随检测噪声漂移(3.4→4.1m)，
   致命带跟着机器人走 -> 规划反复重算、机器人卡死。builder 现把环转到 map(退 odom)
   系并冻结几何，新提案质心偏离 >lock_radius_m(0.7) 才重锁。
2. **Layer 脏标记重画**：几何未变不 restore+refill(发布间隙闪烁)，兜底 refresh_period 2s
   应对滚动窗。
3. **负候选必须"穿透见后墙"**：候选列无实体回波(vr=-1)一律排除 —— 否则超出激光量程的
   真墙(东墙 11.8m)会被报成玻璃，把机器人围死。
4. **脚线取"最深回波簇"**(percentile 90)：面板框/柱回波(v≈165)远深于穿透后墙回波
   (v≈135)，75% 分位会被后墙大簇淹没(带锚到 11.8m 的根因)。

## 仿真演示标准流程

合成场景 segformer 分类随姿态摆动(screen/wall/ceiling 互换)，自动掩码不可复现 ->
演示用 manual_roi 作为可信提案源(即 GlassGuard 的"视觉提议"契约位，真机换微调模型/YOLO)：

```bash
ros2 topic pub -r 2 /fp_sem/manual_roi geometry_msgs/msg/PolygonStamped \
  "{polygon: {points: [{x: 90, y: 60}, {x: 550, y: 60}, {x: 550, y: 166}, {x: 90, y: 166}]}}"
```

## 已验证（2026-10-26, factory1 仿真）

- 覆盖权限：向激光报告 free(发布值 0) 的位置发布 glass 面 -> 发布代价 99(致命)；
  water 面 -> 70(高代价不禁止)；远处不受污染
- 分级+TTL：过期自动恢复所覆盖格子；快照合并保留未重投表面
- TF：base_link 面正确转入 map/odom 双 costmap（用最新变换，弃用观测时间戳——
  观测时刻超前 map->odom 会整帧丢弃）
- 端到端：视觉面 -> 覆盖层 -> 规划器实际绕行/阻断（goal 被 99 区域正确拒绝）
- 代价刻度坑：nav2 内部 0-254，非 ROS1 的 0-100；写 100 只是中等代价（会穿！）

## 已知限制（诚实边界）

1. **合成场景前端不可靠**：segformer 对蓝色合成面板/白墙/车体的输出为
   screen(130)/wall(0)/ceiling(5) 大杂烩，负候选会误检东墙等——机制正确，
   实用性取决于计划中早已排期的**微调玻璃语义模型**（掩码源可换 YOLO/人工，不改机制）。
2. 楼梯下行检测的几何验证目前只有掩码包络 v1（riser 周期/高度突变的专门验证器待做）。
3. 积水在 Gazebo11 无镜面物理，仿真只能验逻辑通路。
4. `/fp_guard/surfaces` 按单发布者快照语义设计（多发布者需命名空间隔离）。

## 下一步

1. 真机数据 -> 微调 segformer/YOLO 出可信 glass/water 掩码
2. 楼梯下行验证器（激光 riser 图案 -> 候选，掩码确认）+ 台阶世界模型
3. costmap 四级 traversability 展示层（可选，覆盖层已是硬需求的答案）
