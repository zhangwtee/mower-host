import os
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMessageBox
from ui.const import MOCK_LAUNCH_SH, MOCK_STOP_SH


class MockSlamMixin:

    def _start_mock_mapping(self):
        """Mock 模式：拉起独立 mock 节点 + 真实 rviz2（脚本位于 ~/Desktop/mock_rviz/mock/，不进本仓库）"""
        import subprocess
        if self._mapping:
            self._request_stop_mapping()
            return
        if not os.path.isfile(MOCK_LAUNCH_SH):
            self.log_send("建图: 未找到启动脚本 " + MOCK_LAUNCH_SH)
            QMessageBox.warning(self, "Mock 建图", "未找到启动脚本：\n" + MOCK_LAUNCH_SH)
            return
        # 先清掉可能残留的上一轮 Mock 进程，防止重复拉起
        if os.path.isfile(MOCK_STOP_SH):
            subprocess.run(["bash", MOCK_STOP_SH],
                           env=dict(os.environ, DISPLAY=":0"),
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        self._mock_proc = subprocess.Popen(
            ["bash", MOCK_LAUNCH_SH],
            env=dict(os.environ, DISPLAY=":0"))
        self._mapping = True
        self._map_saved = False
        self._pending_stop_after_save = False
        self._saving = False
        self._btn_start_slam.setText("停止建图")
        self.status_label.setText("状态:\nMock 建图中")
        self.log_send("建图: Mock rviz2 已启动（独立 mock 节点 + rviz2）")
        self._start_mock_watch()

    def _start_mock_watch(self):
        """轮询后台 launch 进程，rviz 被手动关闭时同步复位 UI"""
        if self._mock_watch is None:
            self._mock_watch = QTimer(self)
            self._mock_watch.setInterval(500)
            self._mock_watch.timeout.connect(self._watch_mock_proc)
        self._mock_watch.start()

    def _watch_mock_proc(self):
        if self._mock_proc is not None and self._mock_proc.poll() is not None:
            self._mock_watch.stop()
            self._mock_proc = None
            self._mapping = False
            self._btn_start_slam.setText("开始建图")
            self.status_label.setText("状态:\n就绪")
            self.log_send("建图: Mock rviz2 已退出")

    def _stop_mock_mapping(self):
        """Mock 模式：按 PID 文件停止 mock 节点与本方案 rviz2，不触碰真机进程"""
        import subprocess
        if self._mock_watch is not None:
            self._mock_watch.stop()
        if os.path.isfile(MOCK_STOP_SH):
            try:
                subprocess.run(["bash", MOCK_STOP_SH],
                               env=dict(os.environ, DISPLAY=":0"),
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
            except Exception as e:
                self.log_send("建图: 停止脚本异常 " + str(e))
        if self._mock_proc is not None:
            try:
                self._mock_proc.terminate()
            except Exception:
                pass
            self._mock_proc = None
        self._mapping = False
        self._btn_start_slam.setText("开始建图")
        self.status_label.setText("状态:\n就绪")
        self.log_send("建图: Mock 已停止")
