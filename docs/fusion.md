## 仿真环境
```bash
ros2 launch fp_sim fake_sensor.launch.py                      # 终端1
ros2 launch fp_bringup fusion.launch.py calib:=est.yaml       # 终端2 (或 sim_truth.yaml)
rviz2 -d $(ros2 pkg prefix fp_bringup)/share/fp_bringup/config/fp_debug.rviz   # 终端3
```

仿真实测：segformer 端到端 ~13fps（GPU 推理 18ms），点云打标覆盖率 ~50%（=相机 FOV 几何占比）。

### 真实照片/视频预检（上真机前验证分割质量）
```bash
ros2 run fp_sim image_publisher --path /path/to/工作场景照片.jpg        # 或 .mp4 视频(自动按原帧率循环)
ros2 run fp_perception segformer_node --ros-args -p policy_path:=src/fp_bringup/config/policy.yaml
# rviz 看 /fp/sem_vis: 真实图像输出稳定(实测公路视频: 路面 100% road=绿, 护栏红, 天空灰)
```