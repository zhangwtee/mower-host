"""路径与模式常量：MainWindow 与各 mixin 共用。"""
import os


CONFIG_FILE = os.path.expanduser("~/.割草机_config.json")
VIDEO_DIR = os.path.expanduser("~/Videos/割草机")
ROS_WS = os.path.expanduser("~/mower_ws")
MAP_DIR = os.path.expanduser("~/mower_maps")
RVIZ_CFG = ROS_WS + "/src/mower_nav2/rviz/mower.rviz"
MOCK_RVIZ_DIR = os.path.expanduser("~/Desktop/mock_rviz/mock")
MOCK_LAUNCH_SH = MOCK_RVIZ_DIR + "/launch_mock_rviz.sh"
MOCK_STOP_SH = MOCK_RVIZ_DIR + "/stop_mock_rviz.sh"
