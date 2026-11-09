import os
import datetime
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from ui.const import VIDEO_DIR
from ui.camera_widget import CameraWidget


class CameraMixin:

    def _push_sensor_to_camera(self, vl, vr, batt):
        """传感器数据推送给摄像头控件画曲线"""
        cw = getattr(self, 'camera_widget', None)
        if cw and hasattr(cw, 'set_sensor_data'):
            cw.set_sensor_data(vl, vr, batt)

    def _set_rec_values(self, vl, vr, batt):
        """传感器回调: 直接更新文字数值"""
        self.label_rec_values.setText(
            f"<span style='color:#4ade80'>Vx:{vl:.2f}</span>  "
            f"<span style='color:#60a5fa'>Vz:{vr:.2f}</span>  "
            f"<span style='color:#facc15'>电池:{batt:.1f}V</span>")

    def _flush_rec_plot(self):
        if hasattr(self, '_rec_curve_vx') and self._rec_curve_vx:
            xs = list(range(len(self._rec_vx_buf)))
            self._rec_curve_vx.setData(xs, list(self._rec_vx_buf))
            self._rec_curve_vz.setData(xs, list(self._rec_vz_buf))
            self._rec_curve_b.setData(xs, [b/25.0 for b in self._rec_b_buf])

    def _ensure_rec_camera(self):
        if not hasattr(self, 'rec_camera') or not self.rec_camera or not getattr(self.rec_camera, 'cap', None):
            self.rec_camera = CameraWidget()
            self.rec_camera.start_camera()
            if hasattr(self, 'rec_camera_container') and self.rec_camera_container:
                lay = self.rec_camera_container.layout()
                if lay is None:
                    lay = QVBoxLayout(); self.rec_camera_container.setLayout(lay)
                while lay.count(): lay.takeAt(0)
                lay.addWidget(self.rec_camera, 1)

    def _release_camera(self):
        self._stop_grab_record()
        if self.camera_widget: self.camera_widget.stop_recording(); self.camera_widget.stop_camera(); self.camera_widget = None
        if self.camera_window: self.camera_window.close(); self.camera_window = None

    def _on_camera_window_close(self, event): self._release_camera(); event.accept()

    def _start_grab_record(self, label):
        if getattr(self, '_grab_writer', None): return
        import cv2
        os.makedirs(VIDEO_DIR, exist_ok=True)

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(VIDEO_DIR, f"{label}_{ts}.avi")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self._grab_writer = cv2.VideoWriter(path, cv2.CAP_FFMPEG, fourcc, 15.0, (960, 480))
        if not self._grab_writer.isOpened():
            # FFMPEG 失败则回退默认后端再试一次
            self._grab_writer = cv2.VideoWriter(path, fourcc, 15.0, (960, 480))
        if not self._grab_writer.isOpened():
            self._grab_writer = None; return

        # 弹 Qt 预览窗（CameraWidget 内部会打开摄像头）
        self._open_composite_window()
        self._grab_timer = QTimer(); self._grab_timer.timeout.connect(self._grab_frame)
        self._grab_timer.start(66)  # ~15fps
        self.status_label.setText(f"状态:\n录制中\n{label}")
        self.log_send(f"录像: {path}")

    def _open_composite_window(self):
        """预览窗: 摄像头 + 曲线 + 文字（纯 Qt 显示）"""
        self._composite_win = QWidget()
        self._composite_win.setWindowTitle("录制中")
        self._composite_win.resize(960, 480)
        lay = QHBoxLayout(); lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(0)
        self._composite_win.setLayout(lay)

        # 左: 摄像头 (640x480)
        cam_container = QWidget()
        cam_container.setFixedSize(640, 480)
        cam_container.setLayout(QVBoxLayout())
        cam_container.layout().setContentsMargins(0, 0, 0, 0)
        self._rec_cam_widget = CameraWidget()
        self._rec_cam_widget.start_camera()
        self._rec_cam_widget.on_tag_seen = lambda: self.dock.tag_detected()
        cam_container.layout().addWidget(self._rec_cam_widget)
        lay.addWidget(cam_container, 2)

        # 右: 曲线 + 文字
        rv = QVBoxLayout(); rv.setSpacing(2); rv.setContentsMargins(0, 0, 0, 0)
        self._comp_plot = pg.PlotWidget(); self._comp_plot.setBackground('#14161b')
        self._comp_plot.showGrid(x=True, y=True, alpha=0.3)
        self._comp_curve_vx = self._comp_plot.plot(pen=pg.mkPen('#4ade80', width=2), name='Vx')
        self._comp_curve_vz = self._comp_plot.plot(pen=pg.mkPen('#60a5fa', width=2), name='Vz')
        self._comp_curve_b = self._comp_plot.plot(pen=pg.mkPen('#facc15', width=1), name='Bat')
        rv.addWidget(self._comp_plot, 1)
        self._comp_values = QLabel("Vx:-- Vz:-- 电池:--")
        self._comp_values.setStyleSheet("font-size:16px; font-weight:bold; padding:4px;")
        self._comp_values.setAlignment(Qt.AlignCenter)
        rv.addWidget(self._comp_values)
        rv.addStretch(1)
        lay.addLayout(rv, 1)

        self._composite_win.closeEvent = lambda e: self._stop_grab_record()
        self._composite_win.show()

        # 预览曲线刷新
        self._comp_timer = QTimer(); self._comp_timer.timeout.connect(self._flush_comp_plot)
        self._comp_timer.start(200)

    def _flush_comp_plot(self):
        if not hasattr(self, '_comp_curve_vx') or not self._comp_curve_vx: return
        vl = self._speed_left; vr = self._speed_right
        xs = list(range(len(self._rec_vx_buf)))
        self._comp_curve_vx.setData(xs, list(self._rec_vx_buf))
        self._comp_curve_vz.setData(xs, list(self._rec_vz_buf))
        self._comp_curve_b.setData(xs, [b / 25.0 for b in self._rec_b_buf])
        b = self._rec_b_buf[-1] if self._rec_b_buf else 0
        self._comp_values.setText(
            f"<span style='color:#4ade80'>Vx:{vl:.2f}</span>  "
            f"<span style='color:#60a5fa'>Vz:{vr:.2f}</span>  "
            f"<span style='color:#facc15'>电池:{b:.1f}V</span>")

    def _grab_frame(self):
        """OpenCV 合成录像（从 CameraWidget 缓存取帧，不抢摄像头）"""
        import cv2
        cw = getattr(self, '_rec_cam_widget', None)
        cam = cw.last_frame if cw and hasattr(cw, 'last_frame') else None
        wr = getattr(self, '_grab_writer', None)
        if cam is None or wr is None:
            return
        cam = cv2.resize(cam, (640, 480))

        canvas = np.zeros((480, 960, 3), dtype=np.uint8)
        canvas[:, :640] = cam
        canvas[:, 640:] = [20, 22, 30]

        vl = self._speed_left; vr = self._speed_right
        b = self._rec_b_buf[-1] if self._rec_b_buf else 0
        servo = getattr(self, '_rec_servo', 1500)

        y = 35
        for text, color in [
            (f"Vx: {vl:+.3f} m/s", (74, 222, 128)),
            (f"Vz: {vr:+.3f} m/s", (96, 165, 250)),
            (f"Battery: {b:.1f} V", (250, 204, 21)),
            (f"Servo: {servo} us", (192, 192, 192)),
        ]:
            cv2.putText(canvas, text, (655, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 1, cv2.LINE_AA)
            y += 28

        # 波形区（浅色背景 + 网格 + 三线 Vx/Vz/Battery）
        plot_x, plot_y, plot_w, plot_h = 650, 160, 300, 200
        cv2.rectangle(canvas, (plot_x, plot_y), (plot_x + plot_w, plot_y + plot_h),
                      (40, 42, 50), -1)
        # 网格线
        for gy in range(plot_y + 50, plot_y + plot_h, 50):
            cv2.line(canvas, (plot_x, gy), (plot_x + plot_w, gy), (60, 62, 70), 1)
        for gx in range(plot_x + 50, plot_x + plot_w, 50):
            cv2.line(canvas, (gx, plot_y), (gx, plot_y + plot_h), (60, 62, 70), 1)

        # 图例
        cv2.putText(canvas, "Vx", (plot_x + 5, plot_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (74, 222, 128), 1)
        cv2.putText(canvas, "Vz", (plot_x + 35, plot_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (96, 165, 250), 1)
        cv2.putText(canvas, "Bat", (plot_x + 65, plot_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (250, 204, 21), 1)

        # 画 Vx 线
        vx_buf = list(self._rec_vx_buf)
        n = max(len(vx_buf), 1)
        mid_y = plot_y + plot_h // 2
        for i in range(n - 1):
            x1 = plot_x + int(i * plot_w / n)
            x2 = plot_x + int((i + 1) * plot_w / n)
            y1 = mid_y - int(vx_buf[i] * 120)
            y2 = mid_y - int(vx_buf[i + 1] * 120)
            cv2.line(canvas, (x1, max(plot_y, min(plot_y + plot_h, y1))),
                     (x2, max(plot_y, min(plot_y + plot_h, y2))), (74, 222, 128), 1)
        # 画 Vz 线
        vz_buf = list(self._rec_vz_buf)
        for i in range(n - 1):
            x1 = plot_x + int(i * plot_w / n)
            x2 = plot_x + int((i + 1) * plot_w / n)
            y1 = mid_y - int(vz_buf[i] * 120)
            y2 = mid_y - int(vz_buf[i + 1] * 120)
            cv2.line(canvas, (x1, max(plot_y, min(plot_y + plot_h, y1))),
                     (x2, max(plot_y, min(plot_y + plot_h, y2))), (96, 165, 250), 1)
        # 画 Battery 线（缩放到 0~1 范围）
        if self._rec_b_buf:
            b_max = max(self._rec_b_buf) or 1
            b_buf = [x / b_max for x in self._rec_b_buf]
            for i in range(len(b_buf) - 1):
                x1 = plot_x + int(i * plot_w / len(b_buf))
                x2 = plot_x + int((i + 1) * plot_w / len(b_buf))
                y1 = mid_y - int(b_buf[i] * 80)
                y2 = mid_y - int(b_buf[i + 1] * 80)
                cv2.line(canvas, (x1, max(plot_y, min(plot_y + plot_h, y1))),
                         (x2, max(plot_y, min(plot_y + plot_h, y2))), (250, 204, 21), 1)
        wr.write(canvas)

    def _stop_grab_record(self):
        if hasattr(self, '_grab_timer') and self._grab_timer:
            self._grab_timer.stop()
        if hasattr(self, '_comp_timer') and self._comp_timer:
            self._comp_timer.stop()
        if getattr(self, '_grab_writer', None):
            self._grab_writer.release(); self._grab_writer = None
            self.log_send("录像已保存")
        if hasattr(self, '_composite_win') and self._composite_win:
            self._composite_win.close(); self._composite_win = None
        if hasattr(self, '_rec_cam_widget') and self._rec_cam_widget:
            try: self._rec_cam_widget.stop_camera()
            except: pass
            self._rec_cam_widget = None

        # 关录制窗 = 停当前动作（只有真的停了动作才发急停帧）
        stopped_any = False
        if getattr(self, '_selftest_running', False):
            self._selftest_running = False; self.log_send("自检: 手动停止"); stopped_any = True
        if getattr(self, '_dock_running', False):
            self._dock_running = False; self.dock.stop(); self.log_send("回充: 手动停止"); stopped_any = True
        if stopped_any:
            self._speed_left = 0; self._speed_right = 0
            self.status_label.setText("状态:\n就绪")
            # 同上：只停动作，不触发急停锁存
            self.serial_mgr.stop_motion()

    def open_camera_window(self):
        self.camera_window = QWidget(); self.camera_window.setWindowTitle("摄像头画面"); self.camera_window.resize(800, 600)
        layout = QVBoxLayout(); self.camera_window.setLayout(layout)
        self.camera_widget = CameraWidget(); layout.addWidget(self.camera_widget); self.camera_widget.start_camera()
        self.camera_widget.on_tag_seen = lambda: self.dock.tag_detected()
        self.camera_window.closeEvent = self._on_camera_window_close; self.camera_window.show()
