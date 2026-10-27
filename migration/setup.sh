#!/usr/bin/env bash
# ============================================================================
# fusion_perception 一键部署 (目标: Ubuntu 22.04 原生 或 WSL2)
# 用法:  bash migration/setup.sh
# 幂等: 已装步骤自动跳过。需 sudo。全程不要提前 export PATH=/usr/bin 再 conda activate。
# ============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
echo "[0] 仓库: $REPO"

# ---- 1. apt: ROS2 Humble + Gazebo + 依赖 ------------------------------------
if ! command -v add-apt-repository >/dev/null 2>&1; then
  sudo apt update && sudo apt install -y software-properties-common curl gnupg
fi
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[1] 安装 ROS2 Humble 源..."
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.asc
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.asc] \
http://packages.ros.org/ros2/ubuntu jammy main" | \
       sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
else
  echo "[1] ROS2 Humble 已存在, 跳过源配置"
fi
echo "[1] 安装 apt 包..."
sudo apt update
sudo apt install -y $(grep -vE '^#|^\s*$' "$HERE/apt_packages.txt")

# ---- 2. miniforge (conda) ---------------------------------------------------
if [ ! -d "$HOME/miniforge3" ]; then
  echo "[2] 安装 miniforge..."
  curl -sL -o /tmp/mf.sh \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh"
  bash /tmp/mf.sh -b -p "$HOME/miniforge3"
else
  echo "[2] miniforge 已存在, 跳过"
fi
source "$HOME/miniforge3/etc/profile.d/conda.sh"

# ---- 3. conda 环境 robot_ros2 ----------------------------------------------
if conda env list | grep -q "robot_ros2"; then
  echo "[3] conda 环境 robot_ros2 已存在, 跳过创建"
else
  echo "[3] 创建 conda 环境..."
  conda env create -f "$HERE/environment.yml"
fi
conda activate robot_ros2

# ---- 4. torch (按目标机 CUDA 选版本) ---------------------------------------
if python -c "import torch" 2>/dev/null; then
  echo "[4] torch 已装: $(python -c 'import torch;print(torch.__version__)')"
else
  GPU=$(lspci 2>/dev/null | grep -i "nvidia\|vga.*nvidia" | head -1 || true)
  if [ -z "$GPU" ]; then
    echo "[4] 未检测到 NVIDIA GPU -> 装 CPU 版 torch (推理会慢)"
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
  else
    # RTX 50 系(Blackwell/sm_120) 需 cu128+; 老卡用 cu121。默认 cu124 兼容多数。
    echo "[4] 检测到 GPU, 装 CUDA 版 torch (默认 cu124, 50系请改此行 cu128)"
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
  fi
fi

# ---- 5. colcon build --------------------------------------------------------
echo "[5] 编译工作区..."
cd "$REPO"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# ---- 6. 环境提示 ------------------------------------------------------------
echo "[6] 完成。"
if grep -qi microsoft /proc/version 2>/dev/null; then
  echo "    [WSL2] 检测到 WSL: 每个跑 ROS 的终端需 export FASTDDS_BUILTIN_TRANSPORT=UDPv4"
  if ! grep -q FASTDDS_BUILTIN_TRANSPORT "$HOME/.bashrc"; then
    echo "    可追加到 ~/.bashrc:  echo 'export FASTDDS_BUILTIN_TRANSPORT=UDPv4' >> ~/.bashrc"
  fi
fi
echo "    验收:  conda activate robot_ros2 && cd $REPO && ./script/fp_perception.sh sem"
echo "    然后:  bash migration/verify.sh"
