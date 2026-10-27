# 迁移与复现指南 (fusion_perception)

把"标定 → 仿真 → 感知 → 语义 → GlassGuard"整套搬到另一台 Ubuntu 22.04 / WSL2 机器。

## 0. 资产清单（必须一起走的东西）

| 层 | 位置 | 是否随 git |
|---|---|---|
| 代码(13 包)+脚本+文档 | 本仓库 | ✅ git |
| 玻璃/箱子模型 | `src/fp_factory_sim/models/` | ✅ 已入仓 |
| 标定 yaml | `src/fp_semantic_nav/config/sim_truth_robot1.yaml` | ✅ git |
| SLAM 地图 | `src/fp_factory_sim/maps/` | ✅ git |
| conda 环境定义 | `migration/environment.yml` | ✅ git |
| apt 依赖清单 | `migration/apt_packages.txt` | ✅ git |
| torch 权重 | pip 装（按 GPU）| ❌ 需 setup.sh 第4步 |
| segformer 权重 | HF 联网自动下载 | ❌ 离线见 §4 |
| YOLO 权重 | `yolov8s-worldv2.pt` `weights/clip/` | ❌ gitignore，联网自动下 |

## 1. 源机打包

```bash
cd ~/project/fusion_perception
git add -A && git commit -m "wip"           # 收干净
git bundle create /tmp/fp.bundle --all        # 无 git 服务器时
# 有远端则直接 git push
```

## 2. 目标机一键部署

```bash
git clone /tmp/fp.bundle fusion_perception && cd fusion_perception   # 或 clone 远端
bash migration/setup.sh
```
setup.sh 做：ROS2 Humble 源 + apt 包 → miniforge → conda env → torch(按 GPU) → colcon build。
幂等，重复运行安全。

## 3. 逐层手动版（setup.sh 出问题时对照）

```bash
# 3.1 ROS2 Humble + Gazebo（见 apt_packages.txt）
sudo apt update && sudo apt install -y $(grep -vE '^#|^\s*$' migration/apt_packages.txt)

# 3.2 conda 环境
conda env create -f migration/environment.yml && conda activate robot_ros2

# 3.3 torch 按 GPU CUDA 版本（关键，见 §5 坑表）
python -c "import torch;print(torch.cuda.is_available(), torch.version.cuda)"

# 3.4 编译
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

## 4. 离线权重（目标机不能联网时）

从源机拷这几个目录，放到目标机对应位置：
```
~/.cache/huggingface/hub/models--nvidia--segformer-b2-finetuned-ade-512-512/   # 语义
<repo>/yolov8s-worldv2.pt  <repo>/weights/clip/ViT-B-32.pt                     # YOLO-World
```
能联网则全自动下载，无需拷贝。

## 5. 环境差异与已知坑（务必读）

| 症状 | 根因 | 处理 |
|---|---|---|
| SegVis/Overlay "No Image"，segformer 秒退 | 终端没 `conda activate robot_ros2`（torch 只在 conda 里）| 先 activate；`which python3` 应指 robot_ros2 |
| **activate 前** 执行过 `export PATH=/usr/bin:$PATH` | 遮蔽 conda python，activate 静默失效 | 顺序：先 source ROS，再 conda activate，别再改 PATH |
| WSL2 下服务/TF 超时、costmap "no map" | FastDDS SHM 在 WSL2 损坏 | 每个 ROS 终端 `export FASTDDS_BUILTIN_TRANSPORT=UDPv4`（原生 Ubuntu 不需要）|
| spawn 玻璃报 No module lxml | 系统 python 无 lxml | 在 conda 终端跑 spawn，或用脚本 |
| torch 装不上/推理崩 | GPU 架构与 CUDA 版本不匹配（RTX 50 系需 cu128+）| setup.sh 第4步按 `nvidia-smi` 的 CUDA 选 index-url |
| 机器人被语义带围死/地面标红 | 合成场景 segformer 分类摆动（已知前端限制）| 见 fp_glassguard/README；真机需微调模型 |

## 6. 验收（迁移成功的定义）

```bash
conda activate robot_ros2
./script/fp_perception.sh sem
# 另开终端 spawn 玻璃:
ros2 run gazebo_ros spawn_entity.py -entity glass1 -x 6 -y 0 -z 0 -Y 1.5708 \
  -file src/fp_factory_sim/models/window/model.sdf
ros2 launch fp_glassguard glassguard.launch.py
bash migration/verify.sh        # V1-V5 全 [OK] 即复现成功
```

## 7. 路径参数化说明

- launch 里 conda 解释器已改为读 `FP_PYTHON` 环境变量，默认 `~/miniforge3/envs/robot_ros2/bin/python3`。
  目标机 conda 路径不同则：`export FP_PYTHON=/你的/conda/envs/robot_ros2/bin/python3`
- 脚本用 `~/`（$HOME）展开，换用户名无需改动。
- 玻璃模型路径统一为仓库内 `src/fp_factory_sim/models/`。
