# 标定（相机内参 + 相机-雷达外参）操作文档

## 0. 约定

标定文件格式（fp_common.load_calibration）：
```yaml
image_width/height, K(3x3 行主序), D(畸变)
R_lidar_to_cam   # 雷达系(x前y左z上) -> 相机光学系(x右y下z前)
t_lidar_to_cam   # 光学系原点相对雷达原点的偏移（光学系表达）
```
投影：`p_cam = R·p_lidar + t`，`u = fx·x/z + cx, v = fy·y/z + cy`。

## 1. Gazebo 仿真（fp_factory_sim）：直接用真值，免标定

仿真相机参数在 xacro 里是**已知设计值**：
- 内参：640×384，fx=fy=500，cx=320，cy=192（hfov=2·atan(320/500)=1.1408rad），零畸变
- 外参几何：lidar_link (0.35,0,0.8) 水平；camera_link (0.4,0,0.9)，光轴俯视 10°

已生成真值文件：**`src/fp_factory_sim/config/sim_truth.yaml`**
推导（改 xacro 安装位置后用此公式重新生成）：
```
R = M · Ry(-10°)，  M = [[0,-1,0],[0,0,-1],[1,0,0]]   # 机械系->光学系
t = -R · (C - L)，  C-L = (0.05, 0, 0.1)              # 相机原点-雷达原点(base系)
```
自检：雷达正前方 5m 水平点 (5,0,0) 应投影到 (320,114)（俯视 10° → 地平线在中心上方）。

仿真里跑融合感知：
```bash
ros2 launch fp_bringup fusion.launch.py \
    lidar_topic:=/lidar/points_raw image_topic:=/camera/image_raw \
    calib:=src/fp_factory_sim/config/sim_truth.yaml
rviz2 -d $(ros2 pkg prefix fp_bringup)/share/fp_bringup/config/fp_debug.rviz
```
验收：`/fp/points_policy` 彩色点边应严丝合缝贴在物体表面（错位=标定或 TF 问题）。

## 2. fp_sim（合成传感器）练标定

```bash
ros2 launch fp_sim fake_sensor.launch.py board_motion:=true
# 内参（浏览器 http://localhost:8090，[grab] 15~25 帧后 [quit] 自动解算）
ros2 run fp_calibration camera_intrinsics capture --topic /camera/image_raw --dir ./shots \
    --board 11x8 --square 0.025 --out camera_calib.yaml
# 外参（浏览器 http://localhost:8092，[sample] 15~20 次后 [finish]）
ros2 run fp_calibration board_extrinsics --calib camera_calib.yaml --out est.yaml \
    --board 11x8 --square 0.035
# 验收（vs 真值 sim_truth.yaml；标准: 旋转<1° 平移<2cm 重投影<5px）
ros2 run fp_sim eval_extrinsics est.yaml sim_truth.yaml
```

## 3. 真机标定流程（上真机时）

1. hesai 驱动 + 相机驱动起话题，确认点云 x前y左z上
2. 打印 11×8 棋盘格（方格 ≥40mm 贴硬板）
3. 重复 §2 的三步（话题换真机名）；`--init` 猜测与真值差须 <45°
4. rviz 看 `/fp/points_policy` 边缘贴合度作为最终验收
5. 备用无标定板：`manual_align`（点 ≥8 同名角点 + PnP；光轴方向平移约束弱，dz 误差可达 10cm+）

## 4. 坑位（实测）

- 理想棋盘格对点云有 180° 旋转歧义 → 靠 `--init` 消歧
- eval 时雷达系必须 x 前向（z 前向重投影误差虚高 100 倍）
- gazebo 相机 TF 是机械系 `camera_link`（x 前），光学系旋转已折进 R_lidar_to_cam，
  **不要**再叠加 [−90°,0,−90°]
- 静态 TF 必须一次 `sendTransform([tf1,tf2,...])`（transient_local 历史深度=1）
