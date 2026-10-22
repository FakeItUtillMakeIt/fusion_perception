source /opt/ros/humble/setup.bash
cd ~/project/fusion_perception && source install/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate robot_ros2


ros2 launch fp_factory_sim sim.launch.py world:=$PWD/src/fp_factory_sim/worlds/factory1.world xacro:=$PWD/src/fp_factory_sim/urdf/factory_robot1.xacro
ros2 launch fp_factory_sim ekf.launch.py
ros2 launch fp_factory_sim scan.launch.py

# 终端4：感知栈（zband | patchwork 二选一）
ros2 launch fp_ob_perception obstacle_phase1.launch.py rviz:=true
