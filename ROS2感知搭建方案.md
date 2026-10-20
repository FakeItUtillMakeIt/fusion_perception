# ROS2 搭建方案（基于最新感知模块设计）

> **v2 修订（2026-10-20）**：已并入评审意见——①像素关联改为点云前投影；②竖直玻璃改 mask 地脚线法；③塑料袋改几何规则主导（ADE20K 无该类）；④明确净化分支安全红线；⑤定义与现有 fp_perception 管线的复用关系；⑥补时序"快进慢出"原则、性能预算、仿真验证边界与分阶段实施计划。详见 §9~§11。

> 
> 模块定位：**障碍物感知模块，独立于代价地图**
> 核心目标：多线激光(16/32线)+相机语义分割 + 规则融合；输出`ObstacleArray`，包含`vertical_glass` / `raised_plastic_bag` / `general_obstacle`；附带激光观测净化分支（镜面剔除虚点、积水/平放玻璃/薄塑料袋噪点过滤）
> 环境：ROS2（推荐Jazzy/Humble），多线激光驱动，相机驱动，语义分割推理节点，点云处理、几何校验、多边形提取，自定义消息

## 整体架构总览

两大并行子模块：

1. **观测净化子模块**：只清洗原始激光点云，**不输出障碍物多边形**
   - 镜面：剔除镜面引起的虚假激光点
   - 平放玻璃、浅层积水、平铺/轻微鼓起塑料袋：过滤区域内噪点，判定可通行
2. **障碍物检测子模块**：输出带类别障碍物多边形数组 `ObstacleArray`
   - 语义识别 + 几何校验：竖直玻璃→`vertical_glass`；隆起≥底盘间隙塑料袋→`raised_plastic_bag`
   - 其余激光检出的普通障碍物：统一为`general_obstacle`

数据流顺序：

```
硬件驱动层
├─ 相机 → 图像 + camera_info → 语义分割推理节点 → 各类mask（glass / mirror / water / plastic_bag）
└─ 16/32线激光雷达 → PointCloud2 点云

同步与标定
├─ TF树：base_link ↔ camera_link ↔ laser_link 外参标定
└─ 时间同步：图像、mask、点云 时间戳对齐

感知融合主节点 obstacle_perception_node
├─ 步骤1：点云前投影到像素取语义标签（点→mask 查表，复用 lidar_label_node 投影逻辑），生成语义ROI
│   ※ 修订：像素无深度，不存在"像素反投影成3D"；正确方向是 3D→2D 查表关联
├─ 步骤2：观测净化分支（只作用于检测分支输入，见 §9 安全红线）
│   ├─ mirror mask：删除ROI内镜面反射虚点
│   ├─ flat glass / water / flat plastic mask：过滤该区域零散噪点，不生成障碍物
├─ 步骤3：障碍物检测分支（基于净化后的点云）
│   ├─ 语义ROI几何校验：
│   │   · glass mask：**地脚线法**——mask 下边缘投影到地面生成线段→膨胀成障碍区域，点云回波仅作存在性佐证
│   │     （修订：玻璃透射/镜面导致 ROI 内点云稀少，靠点云平面拟合会漏检）
│   │   · plastic_bag：**几何规则主导**——非地面低凸起+法向杂乱/弱回波+凸起≥底盘间隙 → raised_plastic_bag
│   │     （修订：ADE20K 无 plastic_bag 类，语义只能作可选先验，见 §9）
│   ├─ 普通障碍物：剩余点云做地面分割+聚类，无匹配语义mask → general_obstacle
│   ├─ 类别优先级：语义类别 > general_obstacle（同一区域优先语义类别）
│   ├─ 轮廓提取、多边形简化、重叠多边形合并
└─ 输出自定义消息 ObstacleArray（base_link坐标系，多边形+类别+置信+高度区间）

上层消费端（不属于本模块）
├─ 订阅 ObstacleArray，可自行做碰撞检测 / 转Nav2代价地图
└─ ※ 安全红线：原始点云/scan_low → costmap 的安全回路不经过本模块净化，净化永不消除"安全回路里的障碍"
```

## 1、依赖包清单

```
# ROS2基础
ros-humble-rclcpp ros-humble-rclpy ros-humble-pcl-ros ros-humble-pcl-conversions
ros-humble-tf2-geometry-msgs ros-humble-image-transport ros-humble-cv-bridge
# 点云处理
ros-humble-point-cloud-transport ros-humble-laser-geometry
# 语义推理（二选一）
ros-humble-tensorrt-ros  # GPU加速推理，推荐Jetson/RTX平台
ros-humble-onnxruntime-ros
# 消息相关
ros-humble-vision-msgs
```

## 2、自定义消息 ObstacleArray 定义

新建 `msg/Obstacle.msg`

```
std_msgs/Header header
string class_name
float32 confidence
geometry_msgs/Polygon polygon
float32 min_height
float32 max_height
int32 id                # v2: 跨帧跟踪关联 id
string source           # v2: 来源标记 semantic / geometric / fused
```

新建 `msg/ObstacleArray.msg`

```
std_msgs/Header header
Obstacle[] obstacles
```

> 
> 编译后，节点可以发布 `obstacle_perception/msg/ObstacleArray`

package.xml 增加消息生成依赖：

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

CMakeLists片段：

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
 DEPENDENCIES geometry_msgs std_msgs
)
```

## 3、节点拆分（推荐拆成多个独立ROS2节点，解耦，便于单独调试）

> 
> 建议分节点，而不是单一大节点，方便单独录制bag、单独开关模块

### 节点1：`semantic_infer_node` 语义分割推理节点

输入：

- `/camera/image_raw`
- `/camera/camera_info`
输出：
- `/semantic/mask` 语义分割结果（可使用`vision_msgs/Segmentation`或者图像mask），每个像素类别：glass / mirror / water / plastic_bag / background> 
> 输出mask带上置信图，不是硬二值，方便后续置信阈值过滤

可选：发布类别置信图，方便时序滤波。

> **类别可用性（v2 修订）**：现役 segformer-b2/ADE20K 有 windowpane/glass、mirror、water、floor/carpet 等，**没有 plastic_bag**（近似类仅 handbag，不可靠）。因此：
> - glass/mirror/water：直接用现有 segformer_node 输出 + policy.yaml 映射，**零训练成本**
> - plastic_bag：V1 走**纯几何规则**（见节点3分支B），语义先验留待 V2 微调（需采集塑料袋场景数据，预算 1~2 周，暂不排期）
> - 本节点**直接复用现有 `fp_perception/segformer_node`**（含 smooth_k 时域平滑），不重复建设

### 节点2：`pointcloud_preprocess_node` 点云预处理节点

输入：原始`/points_raw`
功能：

- 去除远距离点、nan点、离群点；
- 地面分割（推荐linefit地面分割，ROS2已有开源移植），分离地面点与非地面候选点云；
输出：
- `/points_filtered` 预处理点云
- `/points_non_ground` 非地面候选点云（给障碍物检测）

### 节点3：核心融合节点 `obstacle_perception_node`（C++推荐，性能优先）

输入：

1. `/semantic/mask` 语义mask + 置信图
2. `/points_filtered` 预处理点云
3. TF：`camera_link` ↔ `base_link` ↔ `laser_link`

**内部两大分支：观测净化 + 障碍物检测**

#### 分支A：观测净化（输出净化后的点云，用于下游检测，不输出障碍物多边形）

1. 语义关联（v2 修订）：点云**前投影**到像素（K·[R|t] + TF），逐点查 mask 得语义标签，按类别圈出 3D ROI（复用 lidar_label_node 已验收的投影链路）
2. mirror：ROI区域内剔除镜面反射虚假点
3. flat glass / water / flat plastic bag：ROI区域过滤零散噪点；**判定可通行，不生成障碍物**

> 输出话题 `/points_cleaned` 净化后点云，作为障碍物检测输入
> **安全红线（v2 新增）**：净化输出仅供检测分支消费；原始 `/points_raw`、`/scan_low`→costmap 的安全链路**绝不接入净化结果**。语义误判最多导致"多报"，绝不允许因净化导致"漏报"。

#### 分支B：障碍物检测（生成ObstacleArray）

输入：`/points_cleaned`

1. glass ROI（v2 修订为**地脚线法**）：
   - 取 glass mask 的**下边缘像素**，逐像素与地面求交投影到 base_link → 得到"地脚线段"序列
   - 连续地脚线段合并→按最小障碍宽度（如 0.3m）膨胀成多边形 → `vertical_glass`
   - ROI 内若有回波点：平面拟合法向接近竖直 → 置信度加成；**无回波不作为排除条件**（玻璃本就少回波）
   - mask 区域悬空（下边缘不在地面，如玻璃隔断上方的窗）→ 判定非竖直障碍，跳过
2. plastic_bag（v2 修订为**几何主导**）：
   - 候选=非地面点簇中：顶面高度在 [地面, 地面+0.4m] 且凸起 90% 分位 ≥ `chassis_gap_threshold` 且平面拟合残差大（形态杂乱）或回波强度异常
   - 满足 → `raised_plastic_bag`；若接入微调模型再叠加语义先验
   - 凸起高度 < 底盘间隙 → 可通行，跳过
3. 其余非地面点云，**不属于任何语义mask区域**：
   - 欧式聚类，聚类提取障碍物点簇
   - 提取2D轮廓，多边形简化，重叠多边形合并
   - 类别固定：`general_obstacle`
4. 类别优先级规则：> 
> 如果一块区域同时命中语义类别 + 普通障碍物聚类，**优先语义类别，不再归类为general_obstacle**
5. 置信度计算：
   - vertical_glass / raised_plastic_bag：`conf = mask_conf * geometry_conf`（语义置信 × 几何校验置信）
   - general_obstacle：来自点云聚类占用置信
6. 输出：`/perception/fused_obstacles`，消息类型 `obstacle_perception/msg/ObstacleArray`

### 节点4（可选，调试可视化）`obstacle_rviz_visual_node`

订阅 `/perception/fused_obstacles`，把ObstacleArray转成`MarkerArray`，RViz显示不同颜色多边形：

- vertical_glass：红色
- raised_plastic_bag：橙色
- general_obstacle：绿色

## 4、关键算法模块与开源组件复用

1. 地面分割：`linefit_ground_segmentation_ros2`，Gitee/GitHub有ROS2移植版本，直接复用，用于地面提取、高度计算
2. 点云平面拟合：PCL `PlaneModelCoefficients` + RANSAC，用来求平面法向、平面高度（玻璃姿态判断）
3. 欧式聚类：PCL EuclideanClusterExtraction，普通障碍物点簇聚类
4. 点云转2D轮廓、多边形简化：PCL提取凸包/凹包 + vtk或者自定义Ramer-Douglas-Peucker多边形简化算法，合并重叠多边形
5. 像素反投影：利用OpenCV + TF2，像素+深度(点云)反投影3D空间ROI

## 5、时序滤波（放在obstacle_perception_node内部）

所有语义mask触发规则，**不使用单帧结果**：

- 维护滑动窗口（例如5帧），连续多帧检出同类mask，才启用观测净化或者障碍物输出；
- 抑制单帧误识别（尤其是积水高光容易瞬时误检）；
- **不对称原则（v2 新增）：障碍快进慢出**——新障碍 1 帧即输出（宁可虚警），障碍消失需 N 帧连续确认才删除（防止语义闪烁导致障碍突然消失）；净化（删点）则相反：**慢进快出**，存疑时保留点。

## 6、Launch 文件组织示例

```
obstacle_perception/
├── launch/
│   ├── perception_bringup.launch.py  # 总启动文件
│   ├── semantic_infer.launch.py
│   ├── pointcloud_preprocess.launch.py
│   └── obstacle_perception.launch.py
├── config/
│   ├── params.yaml  # 所有参数放这里
│       · tf参数、底盘安全间隙阈值
│       · 平面法向角度阈值
│       · mask置信阈值、时序滑动窗口长度
│       · 点云离群滤波参数、聚类阈值
│       · 多边形简化误差阈值
├── msg/
├── src/
└── package.xml
```

`params.yaml` 关键参数示例

```
obstacle_perception_node:
  ros__parameters:
    chassis_gap_threshold: 0.08          # 底盘安全间隙，单位m
    plane_angle_threshold: 75.0          # 判定竖直平面角度阈值
    mask_confidence_thresh: 0.7
    temporal_window_size: 5
    polygon_simplify_epsilon: 0.05
    cluster_tolerance: 0.1
```

## 7、测试方案（ROS2 bag离线 + 实机在线）

1. 离线：录制rosbag（image + camera_info + pointcloud + tf），离线回放调试融合逻辑、调参，验证各类场景
2. RViz可视化：
   - 原始点云 / 净化后点云
   - mask图像
   - obstacle marker多边形（区分颜色）
3. 评估脚本（python）：订阅ObstacleArray，与真值多边形对比，分类别计算召回率/虚警率

## 8、工程风险提示

1. **时间同步**：图像、点云时间戳对齐是核心坑，推荐用`message_filters`做时间同步；
2. **TF标定**：相机与激光外参必须精准，否则mask投影到点云ROI会错位；
3. **多边形合并**：多个相邻障碍物聚类会生成多个小多边形，必须增加重叠融合逻辑，避免碎片化；
4. 镜面虚点剔除：镜面虚点形态多变，V1只做mask ROI区域点删除，不做全点云镜面检测，控制复杂度。

## 9、与现有 fp_perception 管线的关系（v2 新增）

**不重建，只补缺**。现有资产与本方案节点的映射：

| 本方案节点 | 现有资产 | 处置 |
|---|---|---|
| semantic_infer_node | `fp_perception/segformer_node`（含 smooth_k、policy.yaml 分组） | **直接复用**，policy 增加 glass/mirror/water 组映射 |
| mask↔点云关联 | `fp_perception/lidar_label_node`（前投影逐点打标，标定已验收） | **复用投影逻辑**（抽到 fp_common） |
| 点云预处理 | 无（现有仅 z 过滤） | **新建**：linefit 地面分割 + 离群滤波 |
| 几何校验/聚类/多边形 | 无 | **新建**：obstacle_perception_node 核心增量 |
| ObstacleArray 消费 | `traversability_grid`（四级代价） | **并存 A/B**：traversability 继续跑，新增 ObstacleArray→costmap polygon layer，对比后决定去留 |

## 10、验证边界与性能预算（v2 新增）

**仿真能验什么、不能验什么**（Gazebo Classic ray 传感器不模拟透射/镜面反射）：

| 分支 | 仿真验证 | 必须真机 |
|---|---|---|
| general_obstacle（聚类/多边形/评估） | ✅ 完全可验（factory1 真值免费：world 模型位姿已知） | - |
| raised_plastic_bag 几何规则 | ✅ 用托盘/低箱替代物验几何通路 | 现象学 |
| vertical_glass 地脚线法 | ⚠️ 仅验"mask→地脚线→多边形"管线逻辑（world 加玻璃纹理板供分割） | 透射/虚点现象必须真机 bag |
| 净化分支 | ⚠️ 仅逻辑验证 | 镜面/积水必须真机 |

**性能预算**（端到端 ≤300ms）：分割推理 18ms(GPU)/every_n=2 约 66ms + message_filters 同步 ≤100ms + 前投影/几何/聚类 <50ms + 发布 <10ms。评估指标：general_obstacle 召回 ≥95%（仿真真值）、位置误差 ≤10cm、虚警 ≤2 个/分钟、延迟 ≤300ms。

## 11、实施阶段（v2 评审结论）

- **Phase 0**：方案冻结（本版即 v2）；确定包名 `fp_obstacle`，msg 独立小包
- **Phase 1** ✅（2026-10-20 完成）：`fp_ob_msgs` + `fp_ob_perception`（z 带地面分割 V1 + DBSCAN 聚类 + 凸包/RDP 多边形 + id 跟踪 + MarkerArray）；factory1 仿真真值评估：**11/11 区召回、0 虚警**。实测标定参数：eps=0.45（3线1°角分辨率远距切向点距所需）、ground_band=0.12（含地面标线）、min_range=0.9。linefit 留作斜坡升级位
- **Phase 2**：raised_plastic_bag 几何规则分支（仿真用托盘类替代物验证）
- **Phase 3**：vertical_glass 地脚线法 + 净化分支（仿真验逻辑；录真机 bag 验现象）
- **Phase 4**：ObstacleArray→costmap polygon layer 消费端；与 /fp/traversability A/B 对比，定终态

---
*（v1 文末的重复粘贴章节已删除，内容并入上文各节）*
