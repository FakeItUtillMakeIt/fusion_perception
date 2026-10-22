# 融合感知 × 建图导航 —— 状态与操作

## 当前状态一览（2026-10-21）

| 模块 | 状态 | 备注 |
|---|---|---|
| Gazebo 工厂仿真（factory1 + 17 线雷达 4 轮车） | ✅ | sim→ekf→scan→navigation 四 launch |
| SLAM 建图 + 地图保存 | ✅ | `maps/factory1_map.{pgm,yaml}`，世界系对齐 |
| Nav2 定位导航 | ✅ | amcl 自动初始化（出生点 (0,0,0)） |
| 障碍物几何检测（OBB 长方体 + odom 跟踪 + coasting） | ✅ | 详见 perception.md |
| **地图静态过滤（问题①）** | ✅ **已验证** | 开过滤 0 框；(0,4) 投 0.5m 小箱 → 唯一存活框 (0.3,3.8)；删箱框 1s 内消失 |
| YOLO 检测分支（玻璃/塑料袋） | ⬜ 下一步 | 阶段 A：ultralytics 节点 + /fp/detections |
| 语义×几何合流（Phase 2/3） | ⬜ 排队 | 见 ROS2感知搭建方案.md §11 |

## 关键环境要求（WSL2）

```bash
export FASTDDS_BUILTIN_TRANSPORT=UDPv4   # 已写入 ~/.bashrc，新终端自动生效
```
不加会随机出现：lifecycle 激活失败、TF/服务响应丢失、节点收不到 /map——症状千奇百怪，先查它。

## 完整启动（每终端都要 source humble + install + PATH=/usr/bin）

```bash
# 终端1 仿真
ros2 launch fp_factory_sim sim.launch.py \
    world:=$PWD/src/fp_factory_sim/worlds/factory1.world \
    xacro:=$PWD/src/fp_factory_sim/urdf/factory_robot1.xacro  headless:=false
# 终端2 EKF        
ros2 launch fp_factory_sim ekf.launch.py
# 终端3 切片       
ros2 launch fp_factory_sim scan.launch.py
# 终端4 导航+地图  
ros2 launch fp_factory_sim navigation.launch.py map:=$PWD/src/fp_factory_sim/maps/factory1_map.yaml
# 终端5 感知+rviz（conda robot_ros2）
ros2 launch fp_ob_perception obstacle_phase1.launch.py rviz:=true
```

rviz 里可勾选 **Static Map**（/map，Transient Local QoS）查看静态地图底图。
**Fixed Frame 必须设为 `map`**：latched 地图的时间戳停在 map_server 启动瞬间，
rviz 的 TF 过滤器在 odom 系下查不到那么旧的 map→odom 会永久丢弃该消息
（现象：控制台刷 "Message Filter dropping message: frame 'map' ... earlier than all data"）。

## 静态过滤验证（已通过，复测照做）

```bash
# 预期 0 框（全静态）
ros2 topic echo /fp_ob/obstacles --once

# 投放小箱 → 唯一存活框
ros2 run gazebo_ros spawn_entity.py -entity test_crate -x 0 -y 4 -z 0.2 -file /tmp/opencode/mystery_box.sdf
# 删除 → 框 1 秒内消失
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: test_crate}"
```
参数（obstacle_params.yaml，改动需重启 detect）：`static_map_filter: true`、
`static_frac_thresh: 0.6`、`static_occ_level: 50`。

## 历史链路（fp_sim 合成传感器，练标定用，保留）

```bash
ros2 launch fp_sim fake_sensor.launch.py
ros2 launch fp_bringup fusion.launch.py calib:=est.yaml       # 或 sim_truth.yaml
rviz2 -d $(ros2 pkg prefix fp_bringup)/share/fp_bringup/config/fp_debug.rviz
```
仿真实测：segformer ~13fps（GPU 18ms），点云打标覆盖率 ~50%。

真实照片/视频预检：
```bash
ros2 run fp_sim image_publisher --path /path/to/照片.jpg      # 或 .mp4
ros2 run fp_perception segformer_node --ros-args -p policy_path:=src/fp_bringup/config/policy.yaml
```

## 玻璃测试模型（Gazebo Classic 关键结论）

**Gazebo 11 的 CPU ray 传感器基于 collision 几何投射**（`laser_retro` 挂在 collision 上即证据），
`<transparency>` 只影响渲染、对激光完全无效。实测：

| 玻璃面板配置 | 激光 | 相机 | 结论 |
|---|---|---|---|
| visual+collision（原始 window 模型） | ❌ 打中 | ✅ | 不能模拟玻璃 |
| collision-only | ❌ 打中（33点） | 看不到 | 反例 |
| **visual-only（无 collision）** | ✅ **穿透（0点）** | ✅ | **正确配方** |

已按此生成 `/home/sevnce/model_editor_models/window/model_lidar_passthru.sdf`：
玻璃=纯 visual（SkyBlue+transparency 0.7），底框/边柱=visual+collision（激光可见、挡车）。
spawn 用 `-Y 1.5708` 横置，位置自定。代价：机器人物理上可穿过玻璃面（由框柱部分阻挡）。

```bash
ros2 run gazebo_ros spawn_entity.py -entity win_pt -x -4 -y 8 -z 0 -Y 1.5708 \
    -file /home/sevnce/model_editor_models/window/model_lidar_passthru.sdf
```

## 相关文档

- 建图操作：docs/build_map.md
- 导航操作：docs/nav.md
- 标定：docs/calib.md
- 障碍物感知模块：docs/perception.md
- 总体方案与阶段：ROS2感知搭建方案.md（v2）
