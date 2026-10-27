# fp_glassguard — GlassGuard 论文四阶段流水线（独立实现）

## 仿真实测状态（2026-10-26）

四阶段全部激活并验证过单点：柱状候选(sup 35-46)、containment 验证、
假设生命周期(创建 L=3.6m -> miss 累积 -> 支持度衰减 -> drop -> 重建)、
floor/实心测试(东墙类候选 cov=1.0 被拒)。
关键调参教训(都进了默认值): 密网格窗的沿线连续性=实心墙 -> 加"底沿深度一致性"旁路;
验证度量用 containment(投影落在掩码内) 而非 IoU(平面仅覆盖柱支撑段);
近邻簇窗口 0.7m(1.2m 会混入货架); miss 阈值 25 帧(5Hz 下 8 帧过敏)。
干净演示需要: 机器人静止对准玻璃 + ROI 持续供给(遥控窗口请关闭)。

不动任何旧包；输出走既有 `fp_surface_guard.msg.SurfaceArray` 契约，
因此现有 `SurfaceOverrideLayer`（costmap 覆盖层）直接可用。

```
① 掩码提案      /fp/labels 玻璃类 | /fp_sem/manual_roi     (真机=微调实例模型, 接口不变)
② 柱状候选      激光体素垂直柱(≥3高度层) = 框/柱/边稳定证据
                -> PCA 竖直平面, 由柱界定范围(有界! 非无限延伸)
                -> 不拟合玻璃自身回波, 不用单目深度
③ 投影验证      平面四角投回图像 vs 掩码 IoU >= min_iou
                内部空腔: 掩码中部在平面深度带内柱占比 <= max_interior_frac
                (实心墙内部满回波 -> 拒绝; 玻璃内部空 -> 通过)
④ 假设管理      map 系假设表 + 支持度 EMA; 线参数距离/角度合并
                多视衰减: 连续未见/投影落掩码外 -> 衰减直至删除
                FLOOR-EVICT: 条带被连续地面回波占据且无柱支撑 -> 立即驱逐
                (防"地面误标成障碍把机器人围死")
```

## 运行

```bash
./script/fp_perception.sh sem                    # 底座(含 segformer)
ros2 run gazebo_ros spawn_entity.py -entity glass1 -x 6 -y 0 -z 0 -Y 1.5708 \
  -file src/fp_factory_sim/models/window/model.sdf   # conda 终端
ros2 launch fp_glassguard glassguard.launch.py   # 流水线 + 导航(含覆盖层)
# 面板清晰可见时发几秒 ROI(真机换成模型实例掩码即全自动):
ros2 topic pub -r 2 /fp_sem/manual_roi geometry_msgs/msg/PolygonStamped \
  "{polygon: {points: [{x: 90, y: 60}, {x: 550, y: 60}, {x: 550, y: 166}, {x: 90, y: 166}]}}"
```

RViz 加 MarkerArray `/fp_glassguard/markers`：每个假设画 3 层高度线框 +
文字 `glass#id s=支持度 c=置信度`。

## 与旧 surface_builder 的关系

| | fp_surface_guard/surface_builder | fp_glassguard（本包） |
|---|---|---|
| 候选几何 | 掩码底沿→地面投影薄条带 | 柱状证据拟合的有界竖直平面 |
| 验证 | 类别+面积+穿透门控 | 投影 IoU + 内部空腔 + 朝向 |
| 时间维 | 滞回锁（距离阈值） | 支持度衰减/合并/驱逐（证据驱动） |
| 输出 | /fp_guard/surfaces | 同（可互换/共存，二选一发布） |

测试时先杀掉旧 builder 避免双发布者抢快照。
