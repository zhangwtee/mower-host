#!/usr/bin/env bash
# =============================================================================
# setup_pi.sh  树莓派 5B（Ubuntu 24.04 LTS arm64 + ROS 2 Jazzy）板上部署脚本
# 依据：《树莓派Pi5迁移方案 v1.2》4.1 / 4.2 / 4.3 / 4.4 / 6.1–6.4 / 附录 A；台账 1.19 板上实测
# 现役基线：虚拟机 ~/Desktop/mower-host（main.py / core / ui / bridge）+ ~/mower_ws/src 四包
#
# 口径（禁止偏离）：
#   1. 幂等可重跑；apt 已装包自动跳过；不卸载镜像自带 ros-jazzy-desktop 全家桶。
#   2. 不写 ~/.bashrc；三层 overlay source 仅由 .desktop / 启动脚本注入。
#   3. 不自动 pip 安装；Python 依赖全部走 apt，cv2.aruco 缺失仅告警并给兜底命令。
#   4. 上位机源码由 FinalShell 手动上传（虚拟机 → 本机共享文件夹 → SFTP 到 Pi5），
#      脚本只校验目录在位，不内置 scp / rsync / git clone 上位机仓库；
#      rplidar_ros 随源码包上传，保持现役 2.1.4 版本一致，脚本不自动 clone 兜底。
#   5. colcon 单包串行 --executor sequential；不挂 --symlink-install。
#   6. 源码上传排除：install/ build/ log/ __pycache__/ backup/ *.pyc（板上重新生成）。
#   7. 门验收：ros2 结果先落变量、grep 走 here-string；不在 pipefail 下直接管道判断，
#      规避 grep -q 提前退出触发 ros2 BrokenPipeError（退出码 120）造成的误报缺失。
#
# 用法：
#   bash setup_pi.sh              全流程：P1 依赖 → P2 源码校验/rosdep/编译 → P3 权限/规则/启动器
#   bash setup_pi.sh --p1         仅 P1 系统生态包 + Python + rosdep + 门验收
#   bash setup_pi.sh --p2         仅 P2 源码校验 + rosdep + 单包串行编译 + md5 核对
#   bash setup_pi.sh --p3         仅 P3 dialout 组 + udev 规则 + 目录 + .desktop
#   bash setup_pi.sh --check      只读现状探针与门验收，不安装、不改系统、不写文件
#
# 首次执行前：bash -n setup_pi.sh  做语法检查。
# =============================================================================
set -euo pipefail

# ----------------------------- 路径常量（方案 6.2，禁止改动）-----------------------------
HOME_DIR="$HOME"
WS="$HOME_DIR/mower_ws"
WS_SRC="$WS/src"
HOST="$HOME_DIR/Desktop/mower-host"
MAP_DIR="$HOME_DIR/mower_maps"
LOG_DIR="$HOME_DIR/mower_logs"
VIDEO_DIR="$HOME_DIR/Videos/割草机"
BACKUP_DIR="$HOST/backup"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME_DIR/Desktop")"
DESKTOP_FILE="$DESKTOP_DIR/割草机控制面板.desktop"

# ----------------------------- 日志辅助 -----------------------------
c_step() { printf '\n==================== %s ====================\n' "$*"; }
c_ok()   { printf '\033[32m[OK]\033[0m   %s\n' "$*"; }
c_warn() { printf '\033[33m[警告]\033[0m %s\n' "$*"; }
c_err()  { printf '\033[31m[错误]\033[0m %s\n' "$*" >&2; }

MISS=(); WARN=()
add_miss() { MISS+=("$1"); c_warn "源码缺失：$1"; }
add_warn() { WARN+=("$1"); c_warn "$1"; }

# ROS2 setup 链自身不兼容 set -u（Jazzy setup.bash 引用未导出变量会触发 unbound variable），
# source 前后临时关闭 nounset；overlay 文件缺失时静默跳过，不中断部署。
ros_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1" 2>/dev/null
  set -u
  return 0
}

# ----------------------------- 安装清单（方案 4.1 A/B 组 + 4.2 Python）-----------------------------
# A 组：Nav2 全家桶 + 建图 + 融合 + 关节状态（板上 1.19 实测缺失项，已装包 apt 自动跳过）
ROS_NAV2_PKGS=(
  ros-jazzy-nav2-bringup
  ros-jazzy-nav2-map-server
  ros-jazzy-nav2-amcl
  ros-jazzy-nav2-controller
  ros-jazzy-nav2-costmap-2d
  ros-jazzy-nav2-bt-navigator
  ros-jazzy-nav2-smac-planner
  ros-jazzy-nav2-planner
  ros-jazzy-nav2-rviz-plugins
  ros-jazzy-nav2-simple-commander
  ros-jazzy-nav2-waypoint-follower
  ros-jazzy-nav2-mppi-controller
  ros-jazzy-slam-toolbox
  ros-jazzy-robot-localization
  ros-jazzy-joint-state-publisher
)
# B 组：消息 / TF / 图像 / 运行时（1.19 实测多数在位，显式列出由 apt 幂等兜底）
ROS_CORE_PKGS=(
  ros-jazzy-std-msgs ros-jazzy-std-srvs
  ros-jazzy-sensor-msgs ros-jazzy-sensor-msgs-py
  ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-map-msgs
  ros-jazzy-diagnostic-msgs ros-jazzy-trajectory-msgs ros-jazzy-control-msgs
  ros-jazzy-action-msgs ros-jazzy-common-interfaces ros-jazzy-rosgraph-msgs
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-ros-py ros-jazzy-tf2-py
  ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-msgs ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-tools
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-image-geometry
  ros-jazzy-laser-geometry ros-jazzy-message-filters
  ros-jazzy-rviz2 ros-jazzy-robot-state-publisher
  ros-jazzy-rclpy ros-jazzy-rclcpp ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-ros2cli
)
# 构建工具链（方案 4.1 末节）
BUILD_TOOLS=(
  python3-colcon-common-extensions python3-rosdep python3-vcstool
  git curl usbutils build-essential cmake g++ make
  python3-pip python3-venv
)
# Python 依赖（方案 4.2，现役全 apt，pip 命名空间为空）
PY_APT_PKGS=(
  python3-pyqt5 python3-pyqtgraph python3-numpy python3-serial python3-opencv
)

# ----------------------------- 0. 环境前置 -----------------------------
env_check() {
  c_step "0. 板上环境校验"
  local arch; arch="$(uname -m)"
  if [ "$arch" != "aarch64" ]; then
    c_err "架构为 $arch，本脚本面向树莓派 aarch64；当前机器非 Pi5，中止。"
    exit 1
  fi
  c_ok "架构 aarch64"
  local distro; distro="$(lsb_release -rs 2>/dev/null || echo unknown)"
  c_ok "Ubuntu $distro（目标 24.04 noble）"
  local mem; mem="$(free -h | awk '/^Mem:/ {print $2}')"
  c_ok "内存 $mem（目标 16GB 档）"
  c_ok "当前用户 $USER（目标同名 zwt；路径口径见 6.2）"
  echo "家目录: $HOME_DIR"
  echo "工作区: $WS"
  echo "上位机: $HOST"
}

# ----------------------------- P1. 系统生态包 + Python + rosdep -----------------------------
p1_install() {
  c_step "P1. 系统生态包 / Python 依赖 / rosdep"
  sudo -v
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends "${BUILD_TOOLS[@]}"
  sudo apt-get install -y "${ROS_NAV2_PKGS[@]}" "${ROS_CORE_PKGS[@]}"
  sudo apt-get install -y "${PY_APT_PKGS[@]}"

  # rosdep 初始化（幂等：规则文件存在则跳过 init）
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  # 国内镜像：源列表 raw.githubusercontent.com 切清华 github-raw 反代（幂等，仅系统层 rosdep 列表）
  sudo sed -i 's#https://raw.githubusercontent.com/ros/rosdistro/master#https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master#g' /etc/ros/rosdep/sources.list.d/20-default.list 2>/dev/null || true
  # 注释 fuerte（2012 EOL 发行版 gbpdistro，Jazzy 不使用；避免国内网络握手超时，幂等）
  sudo sed -i '/fuerte\.yaml/ s/^#*/# /' /etc/ros/rosdep/sources.list.d/20-default.list 2>/dev/null || true
  # 索引走清华 rosdistro 镜像（仅脚本进程环境，不写 .bashrc）；个别 EOL 源超时不阻断
  export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
  rosdep update || add_warn "rosdep update 部分源失败，核心索引已缓存，人工核对"

  p1_verify
}

p1_verify() {
  c_step "P1 门验收"
  # shellcheck disable=SC1091
  ros_source /opt/ros/jazzy/setup.bash
  echo "ROS_DISTRO=${ROS_DISTRO:-未知}"
  # 门验收口径：ros2 pkg list 单次落变量，grep 走 here-string 无管道；
  # pipefail 下 grep -q 提前退出会让 ros2（Python BrokenPipeError）非零退出，管道误判缺失。
  local pkgs
  pkgs="$(ros2 pkg list 2>/dev/null || true)"
  for p in nav2_bringup slam_toolbox robot_localization; do
    if grep -qx "$p" <<<"$pkgs"; then
      c_ok "ROS 包在位：$p"
    else
      add_warn "ROS 包缺失：$p"
    fi
  done
  python3 - <<'PY'
import importlib
for m in ["PyQt5", "pyqtgraph", "numpy", "cv2", "serial", "rclpy"]:
    try:
        mod = importlib.import_module(m)
        print(f"[OK] Python {m} {getattr(mod, '__version__', '')}".strip())
    except Exception as e:
        print(f"[警告] Python {m} 导入失败: {e}")
# aruco 专项探针：缺失才按 4.2 兜底，不自动 pip
try:
    import cv2.aruco
    print("[OK] cv2.aruco 在位")
except Exception as e:
    print(f"[警告] cv2.aruco 缺失: {e}")
    print("       兜底（PEP 668，确认后手动执行）：")
    print("       pip3 install --break-system-packages opencv-contrib-python-headless")
PY
}

# ----------------------------- P2. 源码校验 + rosdep + 编译 -----------------------------
ensure_dirs() {
  mkdir -p "$WS_SRC" "$MAP_DIR" "$LOG_DIR" "$VIDEO_DIR" "$BACKUP_DIR"
}

p2_check_sources() {
  c_step "P2.1 源码目录校验（FinalShell 手动上传，脚本不自动传输）"
  ensure_dirs
  local required=(
    "$HOST/main.py"
    "$HOST/core"
    "$HOST/ui"
    "$HOST/bridge/package.xml"
    "$WS_SRC/mower_bringup/package.xml"
    "$WS_SRC/mower_slam/package.xml"
    "$WS_SRC/mower_nav2/package.xml"
    "$WS_SRC/rplidar_ros/package.xml"
  )
  for item in "${required[@]}"; do
    if [ -e "$item" ]; then c_ok "在位：$item"; else add_miss "$item"; fi
  done
  if [ "${#MISS[@]}" -gt 0 ]; then
    c_warn "以下源码尚未上传，对应包将跳过编译。上传目标与排除规则见脚本头部注释。"
  fi
}

p2_rosdep() {
  c_step "P2.2 rosdep 依赖解析"
  # shellcheck disable=SC1091
  ros_source /opt/ros/jazzy/setup.bash
  local xml_list
  xml_list="$(find "$WS_SRC" -maxdepth 2 -name package.xml 2>/dev/null || true)"
  if [ -d "$WS_SRC" ] && [ -n "$xml_list" ]; then
    rosdep install --from-paths "$WS_SRC" --ignore-src -r -y || add_warn "rosdep 解析 mower_ws 有告警，人工核对缺失包"
  fi
  if [ -f "$HOST/bridge/package.xml" ]; then
    rosdep install --from-paths "$HOST/bridge" --ignore-src -r -y || add_warn "rosdep 解析 bridge 有告警"
  fi
}

build_one() {
  local pkg="$1"
  local wsdir pkgdir
  case "$pkg" in
    mower_bridge) wsdir="$HOST/bridge"; pkgdir="$HOST/bridge" ;;
    *)            wsdir="$WS";          pkgdir="$WS_SRC/$pkg" ;;
  esac
  if [ ! -f "$pkgdir/package.xml" ]; then add_miss "$pkgdir/package.xml"; return 0; fi
  c_step "P2.3 编译 $pkg（串行，不挂 --symlink-install）"
  (
    cd "$wsdir"
    # shellcheck disable=SC1091
    ros_source /opt/ros/jazzy/setup.bash
    colcon build --packages-select "$pkg" --executor sequential
  )
  c_ok "$pkg 编译完成"
}

p2_build() {
  c_step "P2.3 单包串行编译：bridge → mower_bringup → mower_slam → mower_nav2 → rplidar_ros"
  build_one mower_bridge
  build_one mower_bringup
  build_one mower_slam
  build_one mower_nav2
  build_one rplidar_ros
  p2_md5_check
  p2_verify
}

p2_md5_check() {
  c_step "P2.4 install 内配置与源码 md5 核对（#58 口径）"
  local pairs=(
    "$WS_SRC/mower_bringup/config/ekf.yaml|$WS/install/mower_bringup/share/mower_bringup/config/ekf.yaml"
    "$WS_SRC/mower_bringup/config/static_tf.yaml|$WS/install/mower_bringup/share/mower_bringup/config/static_tf.yaml"
    "$WS_SRC/mower_bringup/config/lidar.yaml|$WS/install/mower_bringup/share/mower_bringup/config/lidar.yaml"
    "$WS_SRC/mower_bringup/config/lidar_node_params.yaml|$WS/install/mower_bringup/share/mower_bringup/config/lidar_node_params.yaml"
    "$WS_SRC/mower_nav2/config/nav2_params.yaml|$WS/install/mower_nav2/share/mower_nav2/config/nav2_params.yaml"
    "$WS_SRC/mower_nav2/rviz/mower.rviz|$WS/install/mower_nav2/share/mower_nav2/rviz/mower.rviz"
  )
  for pair in "${pairs[@]}"; do
    local src="${pair%%|*}"; local inst="${pair##*|}"
    if [ ! -f "$src" ]; then add_warn "源码配置缺失：$src"; continue; fi
    if [ ! -f "$inst" ]; then add_warn "install 副本缺失（包可能未编译）：$inst"; continue; fi
    local a b; a="$(md5sum "$src" | awk '{print $1}')"; b="$(md5sum "$inst" | awk '{print $1}')"
    if [ "$a" = "$b" ]; then c_ok "md5 一致：$(basename "$src")"; else add_warn "md5 不一致：$src vs $inst"; fi
  done
}

p2_verify() {
  c_step "P2 门验收"
  # 三层叠加：Jazzy → WS → bridge（断链保护，缺失不报错）
  # shellcheck disable=SC1091
  ros_source /opt/ros/jazzy/setup.bash
  ros_source "$WS/install/setup.bash"
  ros_source "$HOST/bridge/install/setup.bash"
  local expected=(mower_bringup mower_slam mower_nav2 rplidar_ros mower_bridge)
  # 门验收口径：结果落变量，grep 走 here-string 无管道（规避 pipefail + SIGPIPE 误判）。
  local pkgs
  pkgs="$(ros2 pkg list 2>/dev/null || true)"
  for p in "${expected[@]}"; do
    if grep -qx "$p" <<<"$pkgs"; then
      c_ok "ROS 包在位：$p"
    else
      add_warn "ROS 包缺失：$p（未编译或源码未上传）"
    fi
  done
  local exes
  exes="$(ros2 pkg executables rplidar_ros 2>/dev/null || true)"
  if grep -q rplidar_node <<<"$exes"; then
    c_ok "rplidar_node 可执行文件在位"
  else
    add_warn "rplidar_node 未检出（rplidar_ros 未编译或源码未上传）"
  fi
}

# ----------------------------- P3. 权限 / udev / 目录 / 启动器 -----------------------------
p3_configure() {
  c_step "P3. dialout 组 / udev 规则 / 目录 / 桌面启动器"
  sudo -v
  # 串口权限（方案 3.2 第 4 步）
  local _grp
  _grp="$(groups "$USER" 2>/dev/null || true)"
  if grep -qw dialout <<<"$_grp"; then
    c_ok "用户已在 dialout 组"
  else
    sudo usermod -aG dialout "$USER"
    c_warn "已加入 dialout 组，注销重新登录后生效"
  fi

  # udev 规则（方案 4.4；波特率在协议/参数内，规则只做别名，不设波特率）
  sudo tee /etc/udev/rules.d/98-mower-chassis.rules >/dev/null <<'RULES'
# 底盘串口（1a86:55d4），串口别名 /dev/mower_chassis，波特率 115200（代码内设定）
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="mower_chassis"
RULES
  sudo tee /etc/udev/rules.d/97-mower-lidar.rules >/dev/null <<'RULES'
# 激光雷达串口（10c4:ea60），串口别名 /dev/mower_lidar，波特率 460800（lidar.yaml 内设定），勿与底盘互换
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="mower_lidar"
RULES
  sudo udevadm control --reload
  sudo udevadm trigger
  c_ok "udev 规则已写入并 reload/trigger"
  ls -l /dev/mower_chassis /dev/mower_lidar 2>/dev/null || c_warn "串口设备未接（Mock 阶段正常，P4 真机接入后自动出别名）"

  ensure_dirs
  c_ok "目录已建：$MAP_DIR $LOG_DIR $VIDEO_DIR $BACKUP_DIR"

  write_desktop
}

write_desktop() {
  c_step "生成桌面启动器 $DESKTOP_FILE"
  local exec_line
  exec_line="bash -c \"[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || true; [ -f $WS/install/setup.bash ] && source $WS/install/setup.bash || true; [ -f $HOST/bridge/install/setup.bash ] && source $HOST/bridge/install/setup.bash || true; cd $HOST && python3 main.py\""
  cat > "$DESKTOP_FILE" <<DESKTOP
[Desktop Entry]
Name=割草机控制面板
Comment=割草机底盘控制台（Jazzy 三层 overlay）
Icon=utilities-terminal
Terminal=false
Type=Application
Categories=Development;
Exec=$exec_line
DESKTOP
  chmod +x "$DESKTOP_FILE"
  c_ok "启动器已生成（三层 source 断链保护沿用现役口径）"
}

# ----------------------------- --check 只读探针 -----------------------------
do_check() {
  c_step "只读现状探针（不安装、不改系统）"
  echo "架构: $(uname -m)"
  echo "系统: $(lsb_release -ds 2>/dev/null)"
  echo "内存: $(free -h | awk '/^Mem:/ {print $2}')"
  echo "磁盘: $(df -h / | awk 'NR==2 {print $2" 已用 "$3" 可用 "$4}')"
  echo "会话: XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-未知} DISPLAY=${DISPLAY:-空} XDG_CURRENT_DESKTOP=${XDG_CURRENT_DESKTOP:-未知}"
  echo "用户组: $(groups)"
  echo "CPU 核数: $(nproc 2>/dev/null || echo 未知)"
  echo "温度: $(vcgencmd measure_temp 2>/dev/null || echo 'vcgencmd 不可用')"
  echo "--- 工具链 ---"
  for c in colcon rosdep vcs git curl lsusb gcc g++ cmake make python3 pip3; do
    if command -v "$c" >/dev/null 2>&1; then echo "[OK] $c -> $(command -v "$c")"; else echo "[缺失] $c"; fi
  done
  echo "rosdep init: $([ -f /etc/ros/rosdep/sources.list.d/20-default.list ] && echo 已初始化 || echo 未初始化)"
  echo "--- 待装包 apt 可解析性 ---"
  apt-cache policy ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization ros-jazzy-joint-state-publisher ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-waypoint-follower 2>/dev/null | grep -E "ros-jazzy|Candidate" || echo "apt 源未解析到候选版本"
  echo "--- ROS2 ---"
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    ros_source /opt/ros/jazzy/setup.bash
    echo "ROS_DISTRO=${ROS_DISTRO:-未知}"
    ros2 pkg list 2>/dev/null | grep -E "nav2_bringup|slam_toolbox|robot_localization|tf2_ros|rplidar_ros|rviz2|nav2_map_server|nav2_amcl|nav2_controller|nav2_costmap_2d|mower_bringup|mower_slam|mower_nav2|mower_bridge" | sort
  else
    c_warn "/opt/ros/jazzy/setup.bash 不存在"
  fi
  echo "--- Python ---"
  python3 - <<'PY'
import importlib
for m in ["PyQt5", "pyqtgraph", "numpy", "cv2", "serial", "rclpy", "cv2.aruco"]:
    try:
        mod = importlib.import_module(m)
        print(f"[OK] {m} {getattr(mod, '__version__', '')}".strip())
    except Exception as e:
        print(f"[缺失] {m}: {e}")
PY
  echo "--- 源码目录 ---"
  for d in "$WS" "$WS_SRC" "$HOST" "$MAP_DIR" "$LOG_DIR"; do
    if [ -e "$d" ]; then echo "在位: $d"; else echo "缺失: $d"; fi
  done
  echo "--- 串口 ---"
  ls -l /dev/ttyUSB* /dev/ttyACM* /dev/mower_* 2>/dev/null || echo "无串口设备（Mock 阶段正常）"
}

# ----------------------------- 收尾汇总 -----------------------------
summary() {
  c_step "部署汇总"
  if [ "${#MISS[@]}" -gt 0 ]; then
    c_warn "缺失项 ${#MISS[@]} 条："
    for m in "${MISS[@]}"; do echo "  - $m"; done
  else
    c_ok "无源码缺失项"
  fi
  if [ "${#WARN[@]}" -gt 0 ]; then
    c_warn "告警 ${#WARN[@]} 条："
    for w in "${WARN[@]}"; do echo "  - $w"; done
  fi
  echo
  echo "后续："
  echo "  1. 注销重新登录，使 dialout 组生效；"
  echo "  2. 桌面会话内双击「割草机控制面板」启动，或：cd $HOST && python3 main.py"
  echo "  3. Mock 空跑保持 core/config.py TEST_MODE=True；真机首轮切 False；雷达单起节点（--params-file lidar_node_params.yaml），勿整组带 use_lidar:=true（会与 UI 叠加成双份 EKF/TF）；"
  echo "  4. 日志目录：$LOG_DIR；地图目录：$MAP_DIR；录像目录：$VIDEO_DIR。"
}

# ----------------------------- 主流程 -----------------------------
main() {
  local mode="${1:-all}"
  case "$mode" in
    --p1) env_check; p1_install; summary ;;
    --p2) env_check; p2_check_sources; p2_rosdep; p2_build; summary ;;
    --p3) env_check; p3_configure; summary ;;
    --check) do_check ;;
    all|"") env_check; p1_install; p2_check_sources; p2_rosdep; p2_build; p3_configure; summary ;;
    *) echo "未知参数: $mode"; echo "用法: bash setup_pi.sh [--p1|--p2|--p3|--check]"; exit 2 ;;
  esac
}

main "$@"
