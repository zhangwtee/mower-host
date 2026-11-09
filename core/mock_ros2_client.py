"""Mock ROS2 客户端 —— 无真实硬件时模拟传感器 / 就绪状态假数据。
旧内嵌假地图线程已移除：建图显示由独立 ~/Desktop/mock_rviz/mock 方案承担。"""
from PyQt5.QtCore import QObject, pyqtSignal
import threading, random, time, math


class MockRos2Client(QObject):
    sensor_updated = pyqtSignal(float, float, float, int)
    status_signal = pyqtSignal(str)
    ack_signal    = pyqtSignal(str)
    estop_signal  = pyqtSignal(bool)
    error_signal  = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._vx = 0.0
        self._vz = 0.0
        self._bat = 25.2
        self._t = 0.0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._mock_loop, daemon=True)
        self._thread.start()
        self._status_thread = threading.Thread(target=self._mock_status, daemon=True)
        self._status_thread.start()

    def open(self): self.start()

    def close(self): self.stop()

    def stop(self):
        self._running = False

    def ready_state(self):
        """Mock：底盘/雷达/EKF 视为在线，建图与导航按需再亮。"""
        return {"chassis": True, "lidar": True, "ekf": True,
                "slam": False, "nav": False}

    def topic_online(self, topic):
        """Mock 模式：不判断真实话题，一律视为在线，避免准备流程卡住。"""
        return True

    def stop_motion(self):
        """Mock：无实际速度通道，空实现。"""
        return

    def send(self, data: bytes):
        # Mock 模式：收到指令直接返回，不产生任何硬件动作
        pass

    # ---------- 传感器假数据 ----------
    def _mock_loop(self):
        """每 50ms 发一次传感器数据"""
        while self._running:
            self._t += 0.05
            self._vx = 0.3 * math.sin(self._t * 0.5) + random.uniform(-0.02, 0.02)
            self._vz = 0.2 * math.sin(self._t * 0.3) + random.uniform(-0.01, 0.01)
            self._bat = max(24.0, self._bat - 0.0001)
            self.sensor_updated.emit(self._vx, self._vz, self._bat, 0)
            time.sleep(0.05)

    def _mock_status(self):
        """模拟就绪检测，逐个上报"""
        time.sleep(1)
        self.status_signal.emit('底盘桥已就绪 → /odom /imu /PowerVoltage')
        time.sleep(1)
        self.status_signal.emit('激光雷达已就绪 → /scan')
        time.sleep(1)
        self.status_signal.emit('EKF 融合已就绪 → /odom_combined')
        time.sleep(1)
        self.status_signal.emit('运动控制已就绪 → /cmd_vel')
        time.sleep(1)
        self.status_signal.emit('舵机控制已就绪 → /servo_cmd')
        time.sleep(2)
        self.status_signal.emit('（Mock 模式：无真实硬件，仅模拟数据）')
