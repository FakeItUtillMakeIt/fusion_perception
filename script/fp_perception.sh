#!/usr/bin/env bash
# FP 一键栈：仿真 + EKF + 切片 + 感知(+可选语义/导航)
# 用法:  ./fp_perception.sh              几何栈(默认)
#        ./fp_perception.sh sem          几何+语义(segformer/overlay)
#        ./fp_perception.sh sem nav      再加 navigation(静态地图过滤生效)
# 注意不能用 set -u：ROS/conda 的 setup.bash 引用未定义变量会直接退出
export FASTDDS_BUILTIN_TRANSPORT=UDPv4     # 非交互 shell 不读 .bashrc，必须显式导出

SEM=false; NAV=false
for arg in "$@"; do
  case "$arg" in sem) SEM=true;; nav) NAV=true;; *) echo "未知参数: $arg (可用: sem nav)"; exit 1;; esac
done

echo "[fp] 清场..."
pkill -9 -f "gzserver|robot_state_pub|pointcloud_to_l|ekf_node|async_slam|amcl|map_server|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|waypoint_follower|smoother_server|lifecycle_manager|obstacle_detect|pointcloud_preprocess|patchwork_ground|segformer|lidar_label|overlay_node|traversability_grid|rviz2" 2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate robot_ros2
P=$PWD/src/fp_factory_sim

cleanup() { echo "[fp] 退出，清场..."; pkill -9 -f "gzserver|robot_state_pub|pointcloud_to_l|ekf_node|obstacle_detect|pointcloud_preprocess|segformer|lidar_label|overlay_node|traversability_grid|amcl|map_server|controller_server|bt_navigator|rviz2" 2>/dev/null; }
trap cleanup EXIT INT TERM

echo "[fp] 启动 sim + ekf + scan ..."
ros2 launch fp_factory_sim sim.launch.py \
    world:=$P/worlds/factory1.world xacro:=$P/urdf/factory_robot1.xacro &
ros2 launch fp_factory_sim ekf.launch.py &
ros2 launch fp_factory_sim scan.launch.py &

if [ "$NAV" = true ]; then
    sleep 25
    echo "[fp] 启动 navigation (静态地图过滤将生效) ..."
    ros2 launch fp_factory_sim navigation.launch.py map:=$P/maps/factory1_map.yaml &
fi

sleep 25
echo "[fp] 启动感知栈 (semantics=$SEM) ..."
ros2 launch fp_ob_perception obstacle_phase1.launch.py rviz:=true semantics:=$SEM &

wait
