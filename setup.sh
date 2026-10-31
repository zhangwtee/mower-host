#!/bin/bash
# 割草机控制面板 —— 环境配置
# 用法: bash setup.sh
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 割草机 控制面板环境配置 ==="
echo "目录: $HERE"

# 1. 安装依赖
echo "[1/4] 检查依赖..."
if ! python3 -c "import serial; import cv2; import pyqtgraph; from PyQt5.QtWidgets import QWidget" 2>/dev/null; then
    echo "安装依赖包..."
    # usbutils 是 core/hw_detector.py 调 lsusb 用的，最小化的 jammy 上没有。
    sudo apt install python3-pyqt5 usbutils -y
    # --break-system-packages 要 pip >= 23.0.1：Bookworm 有，Ubuntu 22.04 的 22.0.2 不认这个选项，
    # 而本脚本是 set -e，写死会让它在 jammy 上当场死在这一行。按能力探测。
    PIP_EXTRA=""
    if pip3 install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
        PIP_EXTRA="--break-system-packages"
    fi
    # opencv 必须走 pip，不能换成 apt 的 python3-opencv：jammy 那份是 4.5.4，
    # 而 core/tag_detector.py 用的是 >= 4.7 的新 aruco API（cv2.aruco.ArucoDetector）。
    pip3 install pyserial opencv-contrib-python-headless pyqtgraph numpy $PIP_EXTRA -i https://pypi.tuna.tsinghua.edu.cn/simple
fi

# 2. 配置 udev 设备别名
# 别名按 USB 芯片 ID 认，不看插在哪个口 —— 换口、重插都还是这个名字。
echo "[2/4] 配置硬件规则..."
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="mower_chassis"' | sudo tee /etc/udev/rules.d/98-mower-chassis.rules > /dev/null
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="mower_lidar"' | sudo tee /etc/udev/rules.d/97-mower-lidar.rules > /dev/null
sudo udevadm trigger

# 3. 串口检测
echo "[3/4] 检测设备..."
[ -e /dev/mower_chassis ] && echo "底盘: /dev/mower_chassis ✔" || echo "底盘: 未检测到 —— 桥会起不来"
[ -e /dev/mower_lidar ] && echo "雷达: /dev/mower_lidar ✔" || echo "雷达: 未检测到"

# 3.5 地图保存目录：map_saver_cli 不会自动创建父目录，缺失会导致存盘失败
mkdir -p "$HOME/mower_maps"

# 3.6 雷达官方源码包：Slamtec rplidar_ros 2.1.4（ros2 分支）。
# apt 仓库 2.1.0 是老移植版（仅 A1/A2/A3/S1、无可执行 rplidar_node、无电机服务），不支持 C1，不走 apt。
# 已克隆/已编译则跳过；GitHub 直连受网络限制时回退 gh-proxy 镜像。
echo "[3.6/4] 检查雷达官方源码包..."
source /opt/ros/jazzy/setup.bash
LIDAR_SRC="$HOME/mower_ws/src/rplidar_ros"
mkdir -p "$HOME/mower_ws/src"
if [ ! -d "$LIDAR_SRC" ]; then
    if ! git clone -b ros2 --depth 1 https://github.com/Slamtec/rplidar_ros.git "$LIDAR_SRC" 2>/dev/null; then
        echo "GitHub 直连失败，回退 gh-proxy 镜像..."
        git clone -b ros2 --depth 1 https://gh-proxy.com/https://github.com/Slamtec/rplidar_ros.git "$LIDAR_SRC"
    fi
fi
if [ ! -d "$HOME/mower_ws/install/rplidar_ros" ]; then
    cd "$HOME/mower_ws"
    colcon --log-base log build --packages-select rplidar_ros
fi

# 4. 桌面图标
# 原来写死 /home/$USER/robot_ui，目录实际叫 mower-host —— 那个图标一直是坏的。
# 桌面目录按 xdg-user-dir 取：中文 locale 的 Ubuntu 上它叫 ~/桌面，写死 ~/Desktop 会落进一个不存在的目录。
echo "[4/4] 桌面图标..."
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_DIR/割草机控制面板.sh" << DESK
#!/bin/bash
export DISPLAY=:0
# 三层 overlay：jazzy 底层 → mower_ws 自研三包 → bridge 通信桥；缺层条件跳过不打断链
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || true
[ -f "$HOME/mower_ws/install/setup.bash" ] && source "$HOME/mower_ws/install/setup.bash" || true
[ -f "$HERE/bridge/install/setup.bash" ] && source "$HERE/bridge/install/setup.bash" || true
cd "$HERE"
python3 main.py
DESK
chmod +x "$DESKTOP_DIR/割草机控制面板.sh"

echo "=== 配置完成 ==="
echo
echo "底盘桥要单独 colcon build 一次（之后只有改桥的代码才需要重来）："
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd $HERE/bridge && colcon build"
echo "自研三包与雷达包首次已随 setup.sh 编译；改源码后手动重编："
echo "  source /opt/ros/jazzy/setup.bash && cd $HOME/mower_ws && colcon --log-base log build --packages-select mower_bringup mower_slam mower_nav2 rplidar_ros"
