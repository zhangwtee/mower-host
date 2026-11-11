import os
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_PLUGIN_PATH", None)
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QLabel, QProgressBar
from PyQt5.QtCore import Qt
from core.config import TEST_MODE
from core.ros2_client import Ros2Client
from core.mock_ros2_client import MockRos2Client
from core.protocol import Protocol
from core.dock_controller import DockController
from ui.main_window import MainWindow


class SplashScreen(QDialog):
    """开机动画/启动检测界面"""
    def __init__(self):
        super().__init__()
        self.setFixedSize(600, 400)
        self.setWindowFlag(Qt.FramelessWindowHint)  # 无边框
        self.setStyleSheet("background-color: #1a1a2e; color: white;")
        
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)
        
        # 检测状态
        self.status = QLabel("正在初始化...")
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet("color: #00d4ff; font-size: 13px;")
        
        # 进度条
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(4)
        self.progress.setStyleSheet("""
            QProgressBar { background-color: #333; border: none; }
            QProgressBar::chunk { background-color: #00d4ff; }
        """)
        
        layout.addStretch()
        layout.addSpacing(30)
        layout.addWidget(self.status)
        layout.addWidget(self.progress)
        layout.addStretch()
        
        self.setLayout(layout)
    
    def set_status(self, text, progress):
        self.status.setText(text)
        self.progress.setValue(progress)
        QApplication.processEvents()


def main():
    app = QApplication(sys.argv)
    
    # 开机动画
    splash = SplashScreen()
    splash.show()
    splash.set_status("正在初始化系统...", 10)
    
    if TEST_MODE:
        # Mock 模式
        import time
        splash.set_status("初始化 UI 组件...", 20)
        time.sleep(0.5)
        splash.set_status("加载 Mock 模拟数据...", 40)
        time.sleep(0.5)
        serial_mgr = MockRos2Client()
        print("=== Mock 模式启动 ===")
        splash.set_status("检测底盘桥... Mock模式跳过", 60)
        time.sleep(0.5)
        splash.set_status("检测激光雷达... Mock模式跳过", 80)
        time.sleep(0.5)
        splash.set_status("系统就绪", 100)
        time.sleep(1)
    else:
        # 真机模式检测
        splash.set_status("初始化 ROS2 节点...", 20)
        serial_mgr = Ros2Client()
        
        splash.set_status("检测底盘桥...", 40)
        import time; time.sleep(2)
        
        splash.set_status("检测激光雷达...", 60)
        time.sleep(1)
        
        splash.set_status("检测 EKF 融合...", 80)
        time.sleep(1)
        
        splash.set_status("系统就绪", 100)
        time.sleep(0.5)
    
    splash.close()
    
    # 进主界面
    protocol = Protocol()
    dock_ctrl = DockController(protocol)
    window = MainWindow(serial_mgr, protocol, dock_ctrl)
    # 窗口按可用桌面取大小并居中，普通窗口，边缘能拖
    # 不用 showMaximized()——最大化状态下 GNOME 不能直接拖边改尺寸，用户会觉得"不能调大小"。
    # 尺寸按屏幕留边推算：可用区各边留 12/8 px，上限 1200×900（大屏也不至于拉太开）。
    _scr = app.primaryScreen().availableGeometry()
    _w = max(640, min(_scr.width() - 24, 1200))
    _h = max(400, min(_scr.height() - 16, 900))
    window.resize(_w, _h)
    window.move(_scr.x() + (_scr.width() - _w) // 2,
                _scr.y() + (_scr.height() - _h) // 2)
    window.show()
    
    dock_ctrl.on_servo_cmd = lambda pulse: serial_mgr.send(
        protocol.build_servo_frame(pulse))
    serial_mgr.sensor_updated.connect(
        lambda l, r, batt, servo: (
            setattr(window, '_speed_left', l),
            setattr(window, '_speed_right', r)))
    serial_mgr.status_signal.connect(window.log_send)
    serial_mgr.start()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
