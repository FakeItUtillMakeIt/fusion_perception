#!/usr/bin/env bash
# 迁移后验收链: 逐级确认标定->感知->语义->glassguard 全通。
# 前置: 已跑 setup.sh, 已 ./script/fp_perception.sh sem, 已 spawn 玻璃。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash && source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORT=UDPv4
pass(){ echo "  [OK] $1"; }; fail(){ echo "  [!!] $1"; }
hz(){ timeout 12 ros2 topic hz "$1" --window 3 2>/dev/null | grep -a average | head -1 | grep -oE "[0-9]+\.[0-9]+" || echo 0; }

echo "V1 仿真/传感:"
[ "$(hz /scan | cut -d. -f1)" != "0" ] && pass "/scan 有数据" || fail "/scan 无 (gz 未起?)"
[ "$(hz /camera/image_raw | cut -d. -f1)" != "0" ] && pass "/camera/image_raw 有" || fail "相机无图"

echo "V2 标定投影:"
ls src/fp_semantic_nav/config/sim_truth_robot1.yaml >/dev/null 2>&1 && pass "标定 yaml 在" || fail "缺标定文件"

echo "V3 语义:"
[ "$(hz /fp/labels | cut -d. -f1)" != "0" ] && pass "/fp/labels 有 (segformer 活)" \
  || fail "/fp/labels 无 -> segformer 崩, 多半 conda 没激活 (which python3 应指 robot_ros2)"

echo "V4 glassguard 提案:"
timeout 6 ros2 topic pub -r 5 /fp_sem/manual_roi geometry_msgs/msg/PolygonStamped \
  "{polygon: {points: [{x:90,y:60},{x:550,y:60},{x:550,y:166},{x:90,y:166}]}}" >/dev/null 2>&1 &
sleep 6
timeout 12 ros2 topic echo /fp_guard/surfaces --once 2>/dev/null | grep -q "class_name" \
  && pass "/fp_guard/surfaces 有表面" || fail "无表面 (机器人没对玻璃? node 没起?)"

echo "V5 覆盖层:"
python3 src/fp_surface_guard/scripts/guard_check.py 2>/dev/null | grep -qE "L3.*OK|lethal" \
  && pass "guard_check L1-L3 通过" || fail "跑 guard_check.py 看哪级断"
echo "完成。全绿即可复现绕行演示。"
