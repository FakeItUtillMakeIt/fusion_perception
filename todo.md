# Factory Simulation Prompts for ROS2 Humble + Gazebo
本文件包含一组可直接粘贴到大模型或交给工程师使用的 Prompt 模板与必要参数/检查清单，用于在 ROS2 Humble + Gazebo classic 上快速生成可与感知流水线（话题：/lidar/points_raw, /camera/image_raw, /camera/camera_info, /imu, use_sim_time=true）无缝对接的仿真资源（xacro/URDF、factory.world、launch、验证脚本、噪声设置等），并在此基础上完成 2D SLAM 建图（slam_toolbox）与 Nav2 导航闭环（Prompt F/G/H 及第十三节流程）。

---

## 一、总体说明（使用前请一并粘贴）
目标环境：ROS2 Humble + Gazebo classic，use_sim_time=true。  
话题与 frame 约定（必须匹配感知节点）：
- LiDAR: topic = `/lidar/points_raw`，frame_id = `lidar_link`，message = `sensor_msgs/PointCloud2`，rate = 10 Hz  
- Camera: topic = `/camera/image_raw`，camera_info = `/camera/camera_info`，frame_id = `camera_link`，encoding = `rgb8`，resolution = `640x384`，rate = 30 Hz  
- IMU: topic = `/imu`，frame_id = `imu_link`，message = `sensor_msgs/Imu`，rate ≥ 200 Hz  
- 2D 切片（SLAM/导航用）: topic = `/scan`，message = `sensor_msgs/LaserScan`，由 `pointcloud_to_laserscan` 从 `/lidar/points_raw` 转换，frame_id = `lidar_link`  
- 里程计: topic = `/odom`，message = `nav_msgs/Odometry`，rate = 30 Hz，由差速驱动插件发布  
- 速度指令: topic = `/cmd_vel`，message = `geometry_msgs/Twist`（Nav2 → 驱动插件）  
机器人根 frame：`base_link`。TF 层级（必须完整）：`map` → `odom`（slam_toolbox 或 amcl 发布），`odom` → `base_link`（差速驱动插件发布），`base_link` → `lidar_link`/`camera_link`/`imu_link`（robot_state_publisher 发布）。  
仿真 ROI 参考：前向 12 m，横向 ±3 m；栅格分辨率 0.05 m。  
IMU 本期 SLAM/导航不消费（online_async + 里程计足够）；保留发布，后续可用 robot_localization 做轮式里程计+IMU 融合（需另装 `ros-humble-robot-localization`）。

---

## 二、Prompt A — 生成 URDF/xacro（robot + sensors）
请将下列文本直接提交给代码/文件生成模型以获得 xacro/URDF：

"请生成一个 ROS2‑compatible xacro / URDF 文件，适用于 ROS2 Humble 与 Gazebo classic。要求如下：
- 机器人为差速轮式底盘（两个驱动轮 + 前端 1 个 caster 万向轮），根 frame 名称为 `base_link`，底盘外接圆半径 ≤ 0.35 m（与 Nav2 robot_radius 一致）。
- caster 摩擦必须置零（`mu1=0, mu2=0, mu_rolling=0`），否则会拖拽导致里程计失真与建图鬼影。
- 添加差速驱动 Gazebo 插件 `libgazebo_ros_diff_drive.so`（`gazebo_ros_pkgs` 已安装）：
  - 订阅 `cmd_vel`（geometry_msgs/Twist）。
  - 发布 `/odom`（nav_msgs/Odometry），update_rate = 30 Hz。
  - 发布 TF `odom` -> `base_link`（`publishOdomTF=true`），`odomFrame=odom`、`baseFrame=base_link`、`wheelSeparation=0.6`、`wheelDiameter=0.15`、`maxWheelTorque=20`（轮参数必须与模型几何严格一致）。  
    **注**：Humble 版插件实际读取 snake_case 标签（`publish_odom_tf/odometry_frame/robot_base_frame/wheel_separation/wheel_diameter/max_wheel_torque/left_joint/right_joint/update_rate`），见第十二节实测坑位。
- 添加三个传感器并挂接到 `base_link`：
  1) 3D LiDAR  
     - link 名称：`lidar_link`，位置：x=0.35, y=0.0, z=0.8（相对于 `base_link`），姿态水平（rpy=0，保证 pointcloud_to_laserscan 能切到完整一条中心线用于 2D SLAM）。  
     - Gazebo 插件：能产出 `sensor_msgs/PointCloud2` 的插件（在 URDF 中配置 `<plugin>`），并将话题 remap/发布为 `/lidar/points_raw`。  
     - 参数示例：freq=10 Hz，min_range=0.2 m，max_range=30 m，range_noise_std=0.02 m。
  2) RGB 相机  
     - link 名称：`camera_link`，位置：x=0.4, y=0.0, z=0.9，俯角 -10°（向下 10°）。  
     - 分辨率：640×384，话题 `/camera/image_raw`（encoding `rgb8`），camera_info -> `/camera/camera_info`。示例内参：fx=500, fy=500, cx=320, cy=192。  
     - 插件：`libgazebo_ros_camera.so`（或等效），并在插件里设置 pixel noise σ≈2.0。
  3) IMU  
     - link 名称：`imu_link`，位置：x=0.0, y=0.0, z=0.5。  
     - 发布 `/imu`，freq=200 Hz，gyro_noise_density=2e-4，accel_noise_density=2e-2。
- 在 URDF 中写明每个传感器的 `frame_id`，并确保 TF chain：`base_link` -> `lidar_link`, `camera_link`, `imu_link`。
- 为每个插件明确 topic 名称（remap 至上面指定话题），并在注释中标注所有参数与噪声值。
- 生成的 xacro 可被 `ros2 launch` + `spawn_entity.py` 调用并 spawn 进 Gazebo。
- 输出完整 xacro/URDF 文件文本。"

---

## 三、Prompt B — 生成 minimal factory.world（场景）
将以下文本提交以生成 Gazebo world (.world/.sdf)：

"请生成一个适配 Gazebo classic 的 minimal `factory.world`（SDF），用于测试工厂场景，要求如下：
- 平整地面（材质 concrete），场地至少 30m x 30m。
- 添加以下静态障碍物（并标注用途）：  
  - 托盘（pallet）：box 1.2 × 1.0 × 0.15 m，位置 x=5.0,y=0.5,z=0.075（验证 step_height/occlusion）。  
  - 箱子堆（stacks）：两个 0.6 × 0.6 × 0.6 m 箱子，位置 x=8.0,y=-1.0,z=0.3（遮挡/障碍）。  
  - 大机器（machine）：box 2.0 × 1.2 × 1.5 m，位置 x=3.5,y=2.0,z=0.75（大障碍与遮挡）。  
  - 低台阶/线槽（trench）：长 2.0 m，宽 0.2 m，高 0.06 m，位置 x=6.0,y=-2.0（验证 step_height 阈值）。  
- 默认照明（ambient + directional），并为模型设置物理参数（friction≈1.0, restitution≈0.0）。
- 为传感器噪声设置 realistic 参数（camera 高斯噪声 σ≈2.0；lidar range noise σ≈0.02）。
- 输出可被 Gazebo 加载的 `.world` 文本，并在注释中说明每个障碍物的测试目的。"

---

## 四、Prompt C — 生成 spawn 与 launch（ros2 launch）
用于生成 launch.py 的 Prompt：

"请生成一个 ROS2 Humble 的 `launch.py`（Python）用于：
- 启动 Gazebo 并加载上面的 `factory.world`（确保 `use_sim_time=True`）。  
- 使用 `spawn_entity.py` 或 `robot_state_publisher` 将上面的 xacro/URDF spawn 到仿真中（支持通过参数指定 `robot_namespace`）。  
- 为插件或话题做 remap，确保仿真发布的 topics 与下列标准话题一致（若插件默认不同要 remap）：  
  - LiDAR -> `/lidar/points_raw`  
  - Camera image -> `/camera/image_raw`  
  - Camera info -> `/camera/camera_info`  
  - IMU -> `/imu`  
- 可选：启动 `rviz2`（demo 配置，包含 PointCloud2, Image, OccupancyGrid, TF）。  
- 可选：启动 `teleop_twist_keyboard`（需 `sudo apt install ros-humble-teleop-twist-keyboard`），用于 SLAM 阶段人工驾驶建图（发 `/cmd_vel`）。  
- 输出完整 `launch.py` 文本，并给出运行示例命令（`ros2 launch ...`）。"

---

## 五、Prompt D — 传感器噪声与真实感（用于 world/plugin）
用于增强仿真真实性的 Prompt：

"请在 `.world` 或 sensor plugin 配置里加入以下噪声与真实感设置：
- LiDAR: range gaussian noise σ=0.02 m；min_range=0.2 m；max_range=30 m；scan_rate=10 Hz；missing_point_prob=0.02。  
- Camera: gaussian pixel noise σ=2.0；exposure auto；（若支持）轻微 motion blur；发布 `camera_info`（fx=500, fy=500, cx=width/2, cy=height/2）。  
- IMU: gyro_noise_density=2e-4，accel_noise_density=2e-2，orientation_covariance 非零小值。  
- World physics: max_step_size=0.002, real_time_update_rate=500（两者乘积=实时倍率！125×0.002 会变成 0.25 倍慢动作，实测坑），gravity=-9.81。  
- 在注释中说明如何调整参数以模拟更差或更好传感器。"

---

## 六、Prompt E — 验证脚本与 quick-check 命令
提交以生成快速验证脚本（bash）：

"请输出一组命令与简短脚本用于仿真启动后快速验证数据流是否正常：
1) 启动 world 与 spawn（示例命令）。  
2) 检查话题存在与数据：  
   - `ros2 topic list | grep lidar`  
   - `ros2 topic echo /lidar/points_raw --once`  
   - `ros2 topic echo /camera/image_raw --once`  
   - `ros2 topic echo /camera/camera_info --once`  
   - `ros2 topic echo /imu --once`  
3) 检查 TF：`ros2 run tf2_tools view_frames.py` 或 `ros2 run tf2_ros tf2_echo /base_link /camera_link`。  
4) 启动感知 pipeline（已部署）并验证 costmap：`ros2 topic echo /traversability_costmap --once`。  
5) rviz2 快速配置建议（添加 PointCloud2, Image, OccupancyGrid, TF），以及记录 bag 的命令：  
   - `ros2 bag record /lidar/points_raw /camera/image_raw /imu /traversability_costmap`。  
输出完整的 bash 命令列表与简要说明。"

---

## 七、Prompt F — 2D 切片转换（pointcloud_to_laserscan，SLAM/导航前置）
> 依赖：`sudo apt install ros-humble-pointcloud-to-laserscan`（当前环境未安装）。slam_toolbox 与 Nav2 costmap 均基于 2D LaserScan，不做此步 SLAM 无法工作。

请将下列文本提交以生成 `scan.launch.py`：

"请生成一个 ROS2 Humble 的 launch.py：用 `pointcloud_to_laserscan` 节点把 `/lidar/points_raw`（PointCloud2, frame `lidar_link`）转换为 `/scan`（LaserScan），供 slam_toolbox 与 Nav2 使用。要求：
- 节点参数：`min_height=-0.05`，`max_height=0.05`（切雷达自身平面附近一条，3D LiDAR 中心线近似过原点，故用相对小值；若实际安装高度需切地面请改用 lidar 系下对应 z 区间并说明换算）。
- `angle_min=-3.1415`，`angle_max=3.1415`，`angle_increment=0.0175`（≈1°），`range_min=0.2`，`range_max=30.0`，`scan_time=0.1`，`transform_tolerance=0.3`。
- 输出 LaserScan 的 `frame_id` 保持 `lidar_link`（由 SLAM 端经 TF 解析到 odom）。
- 所有节点 `use_sim_time=True`；topic 名与 frame 通过 launch argument 可覆盖。
- 支持传入 `robot_namespace`。
- 输出完整 launch.py 文本与一行运行示例，并给出验证命令：`ros2 topic echo /scan --once`。"

---

## 八、Prompt G — SLAM 建图（slam_toolbox）
> 依赖：`ros-humble-slam-toolbox`（已安装）。模式选 `online_async`：仿真中边跑边建图。

请将下列文本提交以生成 `slam.launch.py` + `mapper_params_online_async.yaml`：

"请生成 ROS2 Humble 的 SLAM 建图 launch 与参数文件（基于 slam_toolbox）：
- 节点：`slam_toolbox` 包的 `async_slam_toolbox_node`，参数文件方式加载。
- frame 约定：`map_frame=map`，`odometry_frame=odom`，`base_frame=base_link`，`scan_topic=/scan`（必须与驱动插件/URDF 完全一致，否则 TF 断裂）。
- 关键参数：`resolution=0.05`，`max_range=25.0`，`minimum_travel_distance=0.15`，`minimum_travel_heading=0.2`，`use_scan_matching=true`，`relax_factor=0.19`，`transform_publish_period=0.05`，`buffer_size=50`，`mode=mapping`。
- launch 传入 `use_sim_time:=true`（对 slam 节点与 rviz 同时生效）。
- 可选启动 rviz2（Map=LaserScan/TF/Path 配置）。
- 给出人工建图驾驶说明（teleop_twist_keyboard 或 rviz）与保存地图两种方式：
  1) `ros2 run nav2_map_server map_saver_cli -f factory_map --ros-args -p use_sim_time:=true`（产出 Nav2 可用的 .pgm+.yaml）；
  2) `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: factory_map}"`（可后续 localize 续图）。
- 输出完整 launch.py + yaml + 示例命令。"

验证要点：机器人移动时 `/map` 中墙线单线无重影；`ros2 topic hz /scan` ≈ 10 Hz。

---

## 九、Prompt H — Nav2 导航（定位 → 代价地图 → 规划控制）
> 依赖：`ros-humble-navigation2` + `ros-humble-nav2-bringup`（已安装）。流程：用 Prompt G 保存的 `factory_map` 定位导航；若继续用 slam_toolbox 的 `/map` 直接导航，可临时关闭 amcl 将 slam 设为 localize 模式，但推荐 map_server+amcl 的标准闭环。

请将下列文本提交以生成 `navigation.launch.py` + `nav2_params.yaml`：

"请生成 ROS2 Humble Nav2 的 launch 与完整参数文件，要求：
- 组件：map_server（加载 `factory_map.yaml`）+ amcl + controller_server + planner_server + behavior_server + bt_navigator + waypoint_follower + velocity_smoother + lifecycle_manager；全部 `use_sim_time=true`。
- amcl：`scan_topic=/scan`，`odom_frame=odom`，`base_frame=base_link`，`max_particles=50`（省算力），`min_particles=10`，初始位姿由 rviz '2D Pose Estimate' 或参数给定。
- costmap（global+local）：`resolution=0.05`，`robot_radius=0.35`，`inflation_radius=0.5`；观测源 primary = `scan`（LaserScan max 30 m）；预留注释示例：追加订阅 `/fp/filtered_points`（PointCloud2 语义过滤点云）作为第二 observation source，或接 `/fp/traversability` 的 costmap Layer 插件方案说明（参考 readme '接 Nav2 两种方式'）。
- planner：`NavfnPlanner` 或 `SmacPlanner2D`（tolerate_unknown=false 起步保守）。
- controller：首选 `RegulatedPurePursuitController`（备选 MPPI，注明差异）；`desired_linear_vel=0.5`，限幅与仿真底盘能力一致。
- `cmd_vel` 输出必须为 `/cmd_vel` 且不带命名空间（与驱动插件对齐）。
- 提供 rviz2 nav 配置（Map/Path/Pose/LaserScan/Costmap）与最小验收：发送 3 m 前进 goal，机器人在 factory.world 障碍间无碰撞到达；再用 nav2_simple_commander 给一段 Python 测试脚本。
- 输出完整 launch.py + nav2_params.yaml + 运行/验收命令。"

---

## 十、集成 / 测试 检查清单（必须项）
- 在 launch 中设置 `use_sim_time = true`（非常重要）。  
- 确认 camera plugin 发布 `camera_info`（投影和像素→点映射需要它）。  
- 确认 LiDAR plugin 直接发布 `sensor_msgs/PointCloud2`，或在 pipeline 中添加 laser->pointcloud 转换节点。  
- TF 检查：`base_link` 到 `lidar_link`/`camera_link`/`imu_link` 的静态或动态变换必须存在。  
- 在仿真中加入噪声以减轻仿真与真实世界差异。  
- 如果感知节点话题名不同，使用 remap（launch 或节点参数）对齐。  
- 差速驱动插件已发布 `odom`→`base_link` TF 且订阅 `/cmd_vel`（SLAM/导航的先决条件；缺失则建图不动、导航报错）。  
- caster 摩擦已置零（否则里程计失真 → 建图出现双墙鬼影）。  
- `/scan` 有效：`ros2 topic echo /scan --once` 有角度/距离数据，且 `frame_id=lidar_link` 在 TF 树中。  
- slam_toolbox / amcl 与 URDF/插件三方 frame 命名完全一致（`map/odom/base_link/lidar_link`）。  
- SLAM、Nav2、转换节点全部 `use_sim_time=true`（漏一个节点即报时间戳/外推错误）。  
- 感知 `/fp/traversability` 接入 costmap 为二期扩展：先跑通 SLAM+导航闭环，再 A/B 叠加。

---

## 十一、快速参数（便于直接复制）
- LiDAR: freq=10 Hz, min=0.2 m, max=30 m, noise_std=0.02  
- Camera: res=640×384, fx=500, fy=500, cx=320, cy=192, pixel_noise=2.0, rate=30 Hz  
- IMU: rate=200 Hz, gyro_noise=2e-4, accel_noise=2e-2  
- World physics: max_step_size=0.002, real_time_update_rate=500 (=1x 实时; 乘积决定快慢), gravity=-9.81  
- Diff drive: odom_rate=30 Hz, wheel_separation=0.6 m, wheel_diameter=0.15 m, max_torque=20 N·m, caster mu=0  
- Laserscan 切片: min/max_height=±0.05 m, angle_res≈1°(0.0175 rad), range 0.2~30 m, transform_tolerance=0.3  
- SLAM: mode=online_async, resolution=0.05, max_range=25, min_travel_dist=0.15, min_travel_heading=0.2  
- Nav2: robot_radius=0.35, inflation_radius=0.5, desired_linear_vel=0.5, planner=SmacPlanner2D/NavFn, controller=RPP(备选 MPPI), particles=10~50  
- 保存地图: `ros2 run nav2_map_server map_saver_cli -f factory_map --ros-args -p use_sim_time:=true`  
- 手动驾驶建图: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`（需装 ros-humble-teleop-twist-keyboard）

---

## 十二、常见问题与排查建议
### 实测坑位（fp_factory_sim 已趟平，2026-10 验证）
- Humble 版 `libgazebo_ros_diff_drive.so` 参数是 **snake_case**（`left_joint/right_joint/wheel_separation/publish_odom_tf/odometry_frame/robot_base_frame`），旧 camelCase 会报 "Inconsistent number of joints"。
- `libgazebo_ros_ray_sensor.so` 的 `<output_type>` 必须写全类型名 `sensor_msgs/PointCloud2`（写 `point_cloud2` 报 Invalid）。
- 传感器插件 frame 默认落到根 link（base_link）！必须在 `<plugin>` 内显式 `<frame_name>lidar_link/camera_link/imu_link</frame_name>`，否则 TF 与数据 frame 不符。
- caster 球与两个驱动轮**三点共面接地**会导致轮打滑、车不动：caster 抬高 6mm 让驱动轮承重。
- URDF `<material>` 必须带 `name` 属性，否则 gazebo 转换报 "Visual material must contain a name attribute"。
- `spawn_entity.py` 需要 `python3-lxml`；conda python 环境跑 ros 命令时注意 PATH 前置 `/usr/bin`。
- 矩形对称场地中 amcl 初始位姿 yaw 给错（如 0° vs 真实 77°）会收敛到 **90° 旋转对称的错误位姿**（四方墙扫描完全相同）；initialpose 必须给接近真实的 yaw。
- 物理步进 `max_step_size × real_time_update_rate` 决定实时倍率，配错表现为"仿真 0.25x 慢动作、轮子转车不动"（见 Prompt D）。
- 6cm 线槽：2D 切片看不见但底盘物理过不去——建图路径要避开，或作为感知补盲演示。
- caster 防摇正确配方（A/B 实测）：驱动轮必须独立承重（caster 留 ~3mm 间隙），保持默认 kp=1e6/kd=100；间隙 6mm→3mm 即可把 IMU z 抖动从 ±1.6 降到 ±0.85 且地图零重影。**反例**：caster 触地+minDepth 预载会把车尾垫起、驱动轮欠载打滑 → odom 虚高 10%+ → 建图明显变差；轮子加 minDepth 改变有效滚动半径同样毁 odom 一致性；轮关节加 damping 引入速度滞后。URDF 扩展标签是 camelCase（`minDepth/maxVel`），snake_case 会被静默忽略。
- rviz RobotModel 订阅 `/robot_description` 必须用 **Transient Local** QoS（RSP 是 latched 发布），Volatile 收不到模型。
- slam_toolbox 刷屏 "Message Filter dropping... queue is full"：world 的 `real_time_factor=0`（不限速）导致空闲时仿真超实时、扫描洪峰压垮处理队列；改 `real_time_factor=1` 锁实时即消失。rviz "Trying to create a map of size..." 是地图扩大时重建 swatch 的正常 INFO，可忽略。
- 4 轮布局（2 驱动 + 2 后部转向从动轮）实测配方：转向关节 revolute + **大阻尼 damping=0.3** + 限位 ±1.4 rad + velocity limit 4 + 轮心在转向轴后 3cm（caster trail）+ 轮距 y=±0.345（避开车身）+ 离地 3mm。验收：直行 odom 误差 <2%、急转后完全恢复、建图 9/9 零重影。
  **失败模式**（都试过）：转向阻尼太小（0.002~0.05）→ caster shimmy 甩头翻转到限位锁死，整车卡死走不动；阻尼过大/dissipation → 转向被锁成"马车轴"；转向限位 ±2.8 允许翻转 >90° 是自锁的根源。备选：转向关节直接 fixed（直线完美、转弯欠 9%、无卡死风险）。
- slam_toolbox 自带 rviz 配置只有 Grid 无 Map/LaserScan，"rviz 里看不到东西"多半是这个；本项目已提供 `config/factory_slam.rviz / factory_nav.rviz`。

- 若节点不处理消息或看不到 `/clock`：确认 `use_sim_time=true` 并订阅 `/clock`。  
- 若缺 `camera_info`：投影失败（像素无法映射回点云）。  
- 若 LiDAR 是 `LaserScan`：需要 `laser_scan_to_point_cloud` 或在 URDF 用能直接发布 `PointCloud2` 的插件。  
- 若点云与图像对齐错误：检查相机—LiDAR 外参（TF）与相机内参。  
- SLAM 建不出图 / 地图不动：按 TF 链排查 `ros2 run tf2_tools view_frames.py`（map→odom→base_link→lidar_link 必须连通），再查 `/scan` 时间戳是否跟 `/clock` 一致、slam 节点是否收到 odom TF。  
- "extrapolation into the future" 报错：`transform_tolerance` 偏小（提到 0.3）或某些节点 `use_sim_time` 不一致。  
- 地图出现双墙/鬼影：caster 摩擦未置零、wheel_separation/wheel_diameter 与模型不一致、或 driving cube 打滑；仿真里程计过于理想（无打滑）也会让建图比真机"太好看"，注意迁移落差。  
- Nav2 发 goal 不运动：`ros2 topic info /cmd_vel -v` 确认驱动插件在订阅；检查 lifecycle 是否全部 activate（`ros2 lifecycle get /controller_server`）；costmap 无观测源（查 `/scan` 是否进入 local costmap）。  
- rviz Fixed Frame 设 `map` 后报错/地图不显示：amcl 未启动或未给初始位姿（用 '2D Pose Estimate' 对齐后再导航）。  

---

## 十三、如何使用（SLAM + 导航完整流程）
1. 将 Prompt A–E 粘贴给你的代码生成模型或交给工程师生成仿真文件；运行 launch（确保 use_sim_time=true）；用验证脚本检查话题/TF/数据流。  
2. 用 Prompt F 启动 pointcloud_to_laserscan，`ros2 topic echo /scan --once` 确认切片正常。  
3. 用 Prompt G 启动 slam_toolbox（online_async），teleop 手动低速绕场一圈覆盖全部障碍（含托盘/箱子堆/机器/线槽），rviz 确认墙线无双影后 `map_saver_cli` 保存 `factory_map`。  
4. 用 Prompt H 启动 Nav2（map_server + amcl），'2D Pose Estimate' 初始化位姿，rviz 'Nav2 Goal' 发 3 m 目标点验证自动避障导航；再用 nav2_simple_commander 脚本回归测试。  
5. 将仿真 topics 与感知流水线（/lidar/points_raw, /camera/image_raw, /camera/camera_info, /imu）对接（`fusion.launch.py lidar_topic:=/lidar/points_raw image_topic:=/camera/image_raw`），先用真值/简单阈值替换分割模块做功能验证，再替换真实模型推理。  
6. 感知叠加：以 `/fp/filtered_points` 替换或追加 costmap 点云观测源，或用 Layer 插件叠加 `/fp/traversability` 四级代价，A/B 对比导航行为（先只观察不介入，稳定后启用）。  

> **执行状态（2026-10-16）**：步骤 1–4 已完成 ✅。实现包 `src/fp_factory_sim`（xacro/world/scan/slam/navigation launch + nav2 参数），地图已存 `maps/factory_map.{pgm,yaml}`。实测验收：四面墙单线无鬼影；machine/上层箱正确入图；pallet/线槽低于 0.8m 扫描面不可见（2D 盲点，正是感知层价值）；Nav2 定位收敛后 goal (0,5) 导航 SUCCEEDED。启动顺序：`sim.launch.py` → `scan.launch.py` → （建图）`slam.launch.py` + teleop → `map_saver_cli` → （导航）`navigation.launch.py` + 正确 yaw 的 `/initialpose`。

---

如果你需要，我可以把其中任意一个 Prompt 的输出（xacro、`.world`、`launch.py`、验证脚本）直接渲染成文件并发给你；但你已要求仅生成提示模板，这份 Markdown 文档应能直接喂入其他模型或交与工程师使用。