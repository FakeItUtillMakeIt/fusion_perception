#!/usr/bin/env bash
# glass_perception.sh —— 玻璃感知绕行一键全栈
#   标定(内置真值) -> 仿真 -> 几何感知 -> 语义(segformer+YOLO) -> 地脚线融合 -> 绕行导航
# 用法:
#   ./script/glass_perception.sh            全栈(自动投放玻璃面板)
#   ./script/glass_perception.sh noglass    全栈但不投放玻璃(自己手动摆)
# Ctrl+C 一键全部停止
export FASTDDS_BUILTIN_TRANSPORT=UDPv4
exec 9>/tmp/fp_glass.lock
flock -n 9 || { echo '[glass] 已有一个实例在运行！先关掉旧终端(Ctrl+C)或 pkill -f glass_perception 再启动'; exit 1; }
GLASS=true
[ "$1" = "noglass" ] && GLASS=false

cleanup() {
  echo "[glass] 退出，清场..."
  pkill -9 -f "gzserver|gzclient|robot_state_pub|pointcloud_to_l|ekf_node|async_slam|amcl|map_server|controller_server|planner_server|behavior_server|bt_navigator|velocity_smoother|waypoint_follower|smoother_server|lifecycle_manager|obstacle_detect|pointcloud_preprocess|patchwork_ground|segformer|lidar_label|overlay_node|traversability_grid|glass_detector|yolo_roi|rviz2|spawn_entity" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[glass] 清场..."
cleanup; sleep 2

source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate robot_ros2 || echo "[glass] 警告: conda activate 失败(不影响，detector/yolo 用显式解释器)"
P=$PWD/src/fp_factory_sim

echo "[glass] 1/5 仿真 sim+ekf+scan ..."
ros2 launch fp_factory_sim sim.launch.py \
    world:=$P/worlds/factory1.world xacro:=$P/urdf/factory_robot1.xacro headless:=false>/tmp/fp_glass_sim.log 2>&1 &
ros2 launch fp_factory_sim ekf.launch.py >/tmp/fp_glass_ekf.log 2>&1 &
ros2 launch fp_factory_sim scan.launch.py >/tmp/fp_glass_scan.log 2>&1 &
sleep 30

echo "[glass] 2/5 感知底座 几何+语义(obstacle_phase1 semantics) ..."
ros2 launch fp_ob_perception obstacle_phase1.launch.py semantics:=true >/tmp/fp_glass_obst.log 2>&1 &
sleep 10

echo "[glass] 3/5 玻璃链路 detector + yolo ..."
ros2 launch fp_semantic_nav detector.launch.py >/tmp/fp_glass_det.log 2>&1 &
ros2 launch fp_semantic_nav yolo.launch.py >/tmp/fp_glass_yolo.log 2>&1 &
sleep 5

if [ "$GLASS" = true ]; then
  echo "[glass] 4/5 投放玻璃面板 (6,0) ..."
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: glass1}" >/dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity glass1 -x 6 -y 0 -z 0 -Y 1.5708 \
      -file $PWD/src/fp_factory_sim/models/window/model.sdf >/tmp/fp_glass_spawn.log 2>&1
else
  echo "[glass] 4/5 跳过投放玻璃"
fi

echo "[glass] 5/5 绕行导航 glass_nav + rviz ..."
ros2 launch fp_semantic_nav glass_nav.launch.py >/tmp/fp_glass_nav.log 2>&1 &
sleep 25
rviz2 -d ~/project/fusion_perception/src/fp_semantic_nav/config/glass_all.rviz \
    --ros-args -p use_sim_time:=true >/tmp/fp_glass_rviz.log 2>&1 &

echo ""
echo "[glass] 全栈已起。日志: /tmp/fp_glass_*.log"
echo "[glass] rviz 勾选 Seg Vis / YOLO Vis 可看图像；发目标测试:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo '    "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}}"'
echo "[glass] YOLO 未命中时用手动框兜底(链路等价):"
echo "  ros2 topic pub -r 1 /fp_sem/manual_roi geometry_msgs/msg/PolygonStamped \\"
echo '    "{polygon: {points: [{x: 200, y: 100}, {x: 440, y: 100}, {x: 440, y: 170}, {x: 200, y: 170}]}}"'
echo "[glass] Ctrl+C 停止全部"
wait
