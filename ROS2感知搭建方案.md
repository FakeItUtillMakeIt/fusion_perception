# ROS2 搭建方案（基于最新感知模块设计）

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
├─ 步骤1：mask像素反投影到3D空间，生成ROI空间区域
├─ 步骤2：观测净化分支（修改原始点云）
│   ├─ mirror mask：删除ROI内镜面反射虚点
│   ├─ flat_glass / water / flat_plastic mask：过滤该区域零散噪点，不生成障碍物
├─ 步骤3：障碍物检测分支（基于净化后的点云）
│   ├─ 语义ROI几何校验：
│   │   · glass mask + 平面拟合(法向竖直) → vertical_glass障碍物
│   │   · plastic_bag mask + 凸起高度 ≥底盘间隙 → raised_plastic_bag障碍物
│   ├─ 普通障碍物：剩余点云做地面分割+聚类，无匹配语义mask → general_obstacle
│   ├─ 类别优先级：语义类别 > general_obstacle（同一区域优先语义类别）
│   ├─ 轮廓提取、多边形简化、重叠多边形合并
└─ 输出自定义消息 ObstacleArray（base_link坐标系，多边形+类别+置信+高度区间）

上层消费端（不属于本模块）
└─ 订阅 ObstacleArray，可自行做碰撞检测 / 转Nav2代价地图
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

1. mask反投影：利用内参+TF，把mask像素投影到3D空间，得到每个语义类别对应的ROI点云区域
2. mirror：ROI区域内剔除镜面反射虚假点
3. flat glass / water / flat plastic bag：ROI区域过滤零散噪点；**判定可通行，不生成障碍物**

> 
> 输出话题 `/points_cleaned` 净化后点云，作为障碍物检测输入

#### 分支B：障碍物检测（生成ObstacleArray）

输入：`/points_cleaned`

1. 对glass ROI区域：点云平面拟合，计算平面法向量、平面高度
   - 竖直玻璃（法向接近水平）→ 类别`vertical_glass`，提取轮廓生成多边形
   - 平放玻璃 → 可通行，跳过，不输出障碍物
2. 对plastic_bag ROI区域：计算ROI区域相对地面的凸起高度（取90%分位高度，避免单点噪声）
   - 凸起高度 ≥ 底盘安全间隙 → `raised_plastic_bag`，输出多边形
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
- 抑制单帧误识别（尤其是积水高光容易瞬时误检）。

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

## Markdown 完整章节，直接粘贴进方案文档

```
# ROS2系统搭建方案
> 模块定位：障碍物感知模块，独立于导航代价地图；输出自定义ObstacleArray消息，上层消费端自行处理代价图或碰撞检测。
> 硬件：16/32线多线激光雷达 + RGB相机；ROS2版本：Humble/Jazzy

## 整体架构
模块分为观测净化子模块、障碍物检测子模块；以多线点云+相机语义分割为输入，基于规则融合，输出带类别障碍物多边形数组。
数据流：相机图像→语义推理mask；多线激光→原始点云；经过时间同步、TF坐标转换，送入障碍物感知核心节点；先执行观测净化清洗点云，再做障碍物检测，生成ObstacleArray。

### 消息定义
自定义ROS2消息 Obstacle / ObstacleArray
Obstacle：
- std_msgs/Header header
- string class_name：类别 vertical_glass / raised_plastic_bag / general_obstacle
- float32 confidence：置信度0~1
- geometry_msgs/Polygon polygon：base_link坐标系下2D轮廓
- float32 min_height、max_height：障碍物高度区间

ObstacleArray：Obstacle数组，带header。

## 节点拆分设计（解耦设计，便于单独调试）
1. semantic_infer_node：语义分割推理节点
    输入：相机图像、camera_info；输出像素级语义mask与类别置信图，类别包含glass/mirror/water/plastic_bag。
2. pointcloud_preprocess_node：点云预处理节点
    输入原始点云，去除异常点，使用linefit地面分割分离地面点、非地面候选点云；输出预处理点云与非地面点云。
3. obstacle_perception_node（核心融合节点，C++）
    输入：语义mask、预处理点云、TF变换；内部包含观测净化分支、障碍物检测分支。
    - 观测净化分支：将mask像素反投影到3D空间得到ROI；镜面区域剔除虚假激光点；平放玻璃、积水、平铺/轻微鼓起塑料袋仅过滤噪点，判定为可通行，**不输出障碍物多边形**。
    - 障碍物检测分支：
        ① 玻璃ROI区域：RANSAC平面拟合，计算平面法向。竖直玻璃标记为vertical_glass；平放玻璃判定可通行。
        ② 塑料袋ROI区域：计算ROI相对地面凸起高度；凸起高度≥底盘间隙标记为raised_plastic_bag，否则判定可通行。
        ③ 其余无语义mask的非地面点云：欧式聚类提取障碍物，轮廓提取、多边形简化、重叠合并，统一类别general_obstacle。
    > 类别优先级：语义类别优先级高于general_obstacle；同一区域若命中语义类别，不再归类为通用障碍物。
    时序滤波：滑动窗口多帧校验，连续多帧稳定检出mask才执行规则，抑制单帧误检。
    输出话题：/perception/fused_obstacles（ObstacleArray）
4. obstacle_rviz_visual_node（可选调试节点）：订阅ObstacleArray，转换为MarkerArray在RViz可视化，不同类别使用不同颜色。

## 依赖开源组件
1. linefit_ground_segmentation_ros2：地面分割，计算地面基准、凸起高度
2. PCL库：点云RANSAC平面拟合、欧式聚类、离群点过滤
3. OpenCV + TF2：像素反投影、坐标变换
4. Ramer-Douglas-Peucker算法：多边形轮廓简化；重叠多边形合并逻辑自研

## 参数配置
所有可调参数统一放置config/params.yaml，关键参数：底盘安全间隙、平面竖直判定角度阈值、mask置信阈值、时序滑动窗口大小、点云聚类阈值、多边形简化误差。

## Launch组织
采用分层launch文件，分别启动驱动、语义推理、点云预处理、感知核心节点、可视化节点。

## 测试方案
1. 离线bag回放：录制图像、点云、TF，离线调参、验证场景逻辑。
2. RViz可视化调试：查看原始点云、净化后点云、语义mask、障碍物多边形marker。
3. 指标评估：编写Python评估脚本，订阅ObstacleArray，与真值多边形对比，分类别统计召回率、虚警率。

## 工程注意事项
1. 时间同步：使用message_filters完成图像、点云时间戳对齐；
2. 相机-激光外参标定精度直接影响mask投影ROI准确性；
3. 多边形后处理：增加重叠多边形合并，避免障碍物轮廓碎片化；
4. V1范围约束：仅实现规则式融合，不引入端到端模型融合；观测净化仅处理预定义四类语义场景。
```
