"""MainWindow 基类：Qt 信号 + 三层 overlay 常量（mixin 通过 self. 访问）。"""
import os

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget

from ui.const import ROS_WS


class WindowBase(QWidget):

    save_done = pyqtSignal(bool, str, str)

    # 我们自己的底盘桥，整个替掉了原来的底盘节点。
    # 路径按本文件位置推，换机器/换克隆目录都不用改。
    _bridge_setup_bash = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                      "bridge", "install", "setup.bash")
    # 三层 overlay：Jazzy 底层 → mower_ws 自研包 → 底盘桥；任一缺失条件跳过不打断链
    JAZZY_SETUP = "[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || true"
    ROS_SETUP = (JAZZY_SETUP + " && { [ -f " + ROS_WS + "/install/setup.bash ] && source "
                 + ROS_WS + "/install/setup.bash || true; }")
    BRIDGE_SETUP = (ROS_SETUP + " && { [ -f " + _bridge_setup_bash
                    + " ] && source " + _bridge_setup_bash + " || true; }")
    BRIDGE_LAUNCH = "ros2 launch mower_bridge chassis_bridge.launch.py"
    BRINGUP_LAUNCH = "ros2 launch mower_bringup mower_bringup.launch.py"
