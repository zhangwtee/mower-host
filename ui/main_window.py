import json, os, glob, datetime
import numpy as np
import pyqtgraph as pg
from collections import deque
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QLabel,
    QStackedLayout, QSpacerItem, QSizePolicy, QButtonGroup, QRadioButton,
    QMessageBox, QListWidget, QListWidgetItem, QAbstractItemView
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QTextCursor
from ui.camera_widget import CameraWidget
from core.config import TEST_MODE
from ui.speed_gauge import SpeedGauge

CONFIG_FILE = os.path.expanduser("~/.割草机_config.json")
VIDEO_DIR = os.path.expanduser("~/Videos/割草机")

# 树莓派上的绝对路径，全写在这一次。地图由建图时的 map_saver_cli 写进 MAP_DIR，
# 导航和地图库都从这儿扫；换机器只改这三行。
ROS_WS = os.path.expanduser("~/mower_ws")
MAP_DIR = os.path.expanduser("~/mower_maps")
RVIZ_CFG = ROS_WS + "/src/mower_nav2/rviz/mower.rviz"
# Mock 方案独立目录（不进 mower-host 仓库）：Mock 模式按钮拉起这套脚本
MOCK_RVIZ_DIR = os.path.expanduser("~/Desktop/mock_rviz/mock")
MOCK_LAUNCH_SH = MOCK_RVIZ_DIR + "/launch_mock_rviz.sh"
MOCK_STOP_SH = MOCK_RVIZ_DIR + "/stop_mock_rviz.sh"


class MainWindow(QWidget):

    save_done = pyqtSignal(bool, str, str)

    def __init__(self, serial_mgr, protocol, dock_ctrl):
        super().__init__()
        self.save_done.connect(self._on_save_done)
        self.serial_mgr = serial_mgr
        self.proto = protocol
        self.dock = dock_ctrl
        self._follow_on = False
        self.camera_widget = None
        self.camera_window = None
        self._mapping = False
        self._nav_on = False
        self._map_saved = False
        self._pending_stop_after_save = False
        self._saving = False
        self._bridge_proc = None
        self._bringup_proc = None
        self._mock_proc = None
        self._mock_watch = None
        self._speed_left = 0.0
        self._speed_right = 0.0
        self.config = self._load_config()
        self.init_ui()
        self._rec_vx_buf = deque(maxlen=120); self._rec_vz_buf = deque(maxlen=120); self._rec_b_buf = deque(maxlen=120)
        self.serial_mgr.sensor_updated.connect(
            lambda l, r, batt, servo: (self._waveform.update_values(l, r, batt, servo),
                                       self._rec_vx_buf.append(l), self._rec_vz_buf.append(r),
                                       self._rec_b_buf.append(batt),
                                       setattr(self, '_rec_servo', servo),
                                       self._set_rec_values(l, r, batt),
                                       self._push_sensor_to_camera(l, r, batt))
        )
        self._rec_timer = QTimer(); self._rec_timer.timeout.connect(self._flush_rec_plot); self._rec_timer.start(200)
        self.dock.on_tag_found = self._on_tag_found
        self.dock.on_done = self._on_dock_done
        self._status_timer = QTimer(); self._status_timer.timeout.connect(self._update_status); self._status_timer.start(2000)
        self._speed_timer = QTimer(); self._speed_timer.timeout.connect(self._update_speed); self._speed_timer.start(1000)

        # v3 进程模型：真机启动即拉底盘桥常驻；底层组(bringup)首次进叠加层前懒加载
        if not TEST_MODE:
            self._start_bridge()

    def _load_config(self):
        d = {"hw_mode": 0, "servo": True, "ultrasonic": False}
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE) as f: d.update(json.load(f))
            except: pass
        return d

    def _save_config(self):
        with open(CONFIG_FILE, "w") as f: json.dump(self.config, f, indent=2)

    def show_settings(self):
        self.layout().setContentsMargins(11,11,11,11); self.setWindowTitle("设置"); self.stack_layout.setCurrentWidget(self.settings_widget)
    def show_hw_config(self):
        self.setWindowTitle("硬件配置"); self.hw_save_hint.setText(""); self.stack_layout.setCurrentWidget(self.hw_widget)
    def show_main(self):
        self.layout().setContentsMargins(11,11,11,11); self.setWindowTitle("割草机 控制面板"); self.stack_layout.setCurrentWidget(self.main_widget)
    def show_cache_page(self):
        self.setWindowTitle("清除缓存"); self.stack_layout.setCurrentWidget(self.cache_widget)
    def show_video_page(self):
        self.setWindowTitle("视频管理"); self._refresh_video_list(); self.stack_layout.setCurrentWidget(self.video_widget)
    def show_speed_menu(self):
        self.setWindowTitle("速度显示"); self.stack_layout.setCurrentWidget(self.speed_menu_widget)
    def show_gauge_only(self):
        self.setWindowTitle("速度仪表盘"); self.stack_layout.setCurrentWidget(self.gauge_widget)
    def show_gauge_record(self):
        self.setWindowTitle("录制+显示"); self.layout().setContentsMargins(0,0,0,0); self.stack_layout.setCurrentWidget(self.gauge_rec_widget)
        self._ensure_rec_camera()
        from PyQt5.QtWidgets import QApplication
        QApplication.processEvents()
        w, h = self.width(), self.height()
        self.resize(w+1, h); self.resize(w, h)
    def show_waveform(self):
        self.setWindowTitle("实时波形"); self.stack_layout.setCurrentWidget(self.waveform_widget)

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

    def init_ui(self):
        # fit_tuned 2026-10-26：为 1024x600 面板压缩主页面最小高度 ~70px
        self.resize(1000, 500)
        self.setWindowTitle("割草机 控制面板")
        self.stack_layout = QStackedLayout()
        self.setLayout(self.stack_layout)

        self.main_widget = QWidget()
        main_layout = QVBoxLayout()  # 改成垂直布局，上面状态栏，下面三栏
        self.main_widget.setLayout(main_layout)
        
        # 顶部状态栏：状态指示灯
        status_bar = QHBoxLayout()
        status_bar.addWidget(QLabel("系统状态:"))
        
        self.led_chassis = QLabel("● 底盘")
        self.led_chassis.setStyleSheet("color: #888; font-size: 14px;")
        status_bar.addWidget(self.led_chassis)
        
        self.led_lidar = QLabel("● 雷达")
        self.led_lidar.setStyleSheet("color: #888; font-size: 14px;")
        status_bar.addWidget(self.led_lidar)
        
        self.led_ekf = QLabel("● EKF")
        self.led_ekf.setStyleSheet("color: #888; font-size: 14px;")
        status_bar.addWidget(self.led_ekf)
        
        self.led_slam = QLabel("● 建图")
        self.led_slam.setStyleSheet("color: #888; font-size: 14px;")
        status_bar.addWidget(self.led_slam)
        
        self.led_nav = QLabel("● 导航")
        self.led_nav.setStyleSheet("color: #888; font-size: 14px;")
        status_bar.addWidget(self.led_nav)
        
        status_bar.addStretch()
        main_layout.addLayout(status_bar)
        
        # 下面三栏布局
        content_layout = QHBoxLayout()
        left_layout = QVBoxLayout(); left_layout.setAlignment(Qt.AlignTop)

        self.status_label = QLabel("状态:\n就绪"); self.status_label.setAlignment(Qt.AlignCenter); self.status_label.setStyleSheet("font-size:26px; font-weight:bold;")
        left_layout.addSpacerItem(QSpacerItem(20, 16, QSizePolicy.Minimum, QSizePolicy.Fixed)); left_layout.addWidget(self.status_label); left_layout.addStretch()

        self.btn_selftest = QPushButton("整机自检"); self.btn_mow = QPushButton("开始割草"); self.btn_follow = QPushButton("雷达跟随"); self.btn_dock = QPushButton("自动回充"); self.btn_stop = QPushButton("紧急停止"); self.btn_release = QPushButton("解除急停"); self.btn_release.setEnabled(False)
        for btn in [self.btn_selftest, self.btn_mow, self.btn_follow, self.btn_dock, self.btn_stop, self.btn_release]:
            btn.setMinimumHeight(36); btn.setStyleSheet("font-size:20px;"); left_layout.addWidget(btn)
        # 急停是锁存的（固件 uartx_callback.c:138），不解就是复位前一直停着 ——
        # 所以解除必须是主界面上一个够显眼的按钮，不能藏进菜单。
        self.btn_release.setStyleSheet("font-size:20px; background-color:#1e6b3b; color:white;")
        self.btn_stop.setStyleSheet("font-size:20px; font-weight:bold; background-color:#c0392b; color:white;")
        self.btn_selftest.clicked.connect(self.start_self_test); self.btn_mow.clicked.connect(self.start_mow); self.btn_follow.clicked.connect(self.toggle_follow); self.btn_dock.clicked.connect(self.start_dock); self.btn_stop.clicked.connect(self.stop_all); self.btn_release.clicked.connect(self.release_estop)

        self.btn_record = QPushButton("SLAM地图"); self.btn_record.setMinimumHeight(36); self.btn_record.setStyleSheet("font-size:20px;"); self.btn_record.clicked.connect(self.show_slam_page); left_layout.addWidget(self.btn_record)
        self.btn_settings = QPushButton("设置"); self.btn_settings.setMinimumHeight(36); self.btn_settings.setStyleSheet("font-size:20px;"); self.btn_settings.clicked.connect(self.show_settings); left_layout.addWidget(self.btn_settings)
        content_layout.addLayout(left_layout, 1)

        center_layout = QVBoxLayout(); center_layout.addWidget(QLabel("发送日志")); self.send_log = QTextEdit(); self.send_log.setReadOnly(True); self.send_log.document().setMaximumBlockCount(500); center_layout.addWidget(self.send_log, stretch=1); self.btn_clear_send = QPushButton("清空发送日志"); self.btn_clear_send.clicked.connect(self.clear_send_log); center_layout.addWidget(self.btn_clear_send); content_layout.addLayout(center_layout, 1)
        right_layout = QVBoxLayout(); right_layout.addWidget(QLabel("接收日志")); self.recv_log = QTextEdit(); self.recv_log.setReadOnly(True); self.recv_log.document().setMaximumBlockCount(500); right_layout.addWidget(self.recv_log, stretch=1); self.btn_clear_recv = QPushButton("清空接收日志"); self.btn_clear_recv.clicked.connect(self.clear_recv_log); right_layout.addWidget(self.btn_clear_recv); content_layout.addLayout(right_layout, 1)
        main_layout.addLayout(content_layout)
        self.stack_layout.addWidget(self.main_widget)

        self.settings_widget = self._page("设置", [
            ("切换外设 >", self.show_hw_config), ("打开摄像头", self.open_camera_window),
            ("速度显示 >", self.show_speed_menu), ("清除缓存 >", self.show_cache_page),
            ("返回主界面", self.show_main),
        ]); self.stack_layout.addWidget(self.settings_widget)

        self.hw_widget = QWidget(); hwl = QVBoxLayout(); self.hw_widget.setLayout(hwl)
        t = QLabel("切换外设"); t.setAlignment(Qt.AlignCenter); t.setStyleSheet("font-size:28px; font-weight:bold;"); hwl.addWidget(t); hwl.addWidget(QLabel("选择当前连接的传感器配置"))
        self.rb_normal = QRadioButton("摄像头(普通)"); self.rb_depth = QRadioButton("双目+激光雷达"); self.rb_auto = QRadioButton("自动识别")
        self.hw_group = QButtonGroup(); self.hw_group.addButton(self.rb_normal,0); self.hw_group.addButton(self.rb_depth,1); self.hw_group.addButton(self.rb_auto,2)
        m = self.config.get("hw_mode",0); b = self.hw_group.button(m)
        if b: b.setChecked(True)
        for rb in [self.rb_normal, self.rb_depth, self.rb_auto]: rb.setStyleSheet("font-size:18px;"); hwl.addWidget(rb)
        self.hw_save_hint = QLabel(""); self.hw_save_hint.setStyleSheet("color:green;"); hwl.addWidget(self.hw_save_hint)
        sb = QPushButton("保存配置"); sb.setMinimumHeight(40); sb.clicked.connect(self._save_hw); hwl.addWidget(sb)
        bb = QPushButton("← 返回设置"); bb.setMinimumHeight(40); bb.clicked.connect(self.show_settings); hwl.addWidget(bb)
        self.stack_layout.addWidget(self.hw_widget)

        self.cache_widget = self._page("清除缓存", [("清除视频 >", self.show_video_page), ("清除配置文件", self._clear_config), ("← 返回设置", self.show_settings)]); self.stack_layout.addWidget(self.cache_widget)

        self.video_widget = QWidget(); vl = QVBoxLayout(); self.video_widget.setLayout(vl)
        vt = QLabel("视频管理"); vt.setAlignment(Qt.AlignCenter); vt.setStyleSheet("font-size:28px; font-weight:bold;"); vl.addWidget(vt)
        top = QHBoxLayout(); self.video_count_label = QLabel("共 0 个视频"); top.addWidget(self.video_count_label); top.addStretch()
        self.btn_select_all = QPushButton("全选"); self.btn_select_all.setMinimumHeight(35); self.btn_select_all.clicked.connect(self._toggle_select_all); top.addWidget(self.btn_select_all)
        self.btn_sort_video = QPushButton("时间▼"); self.btn_sort_video.setMinimumHeight(35)
        self.btn_sort_video.clicked.connect(lambda: (setattr(self, '_video_sort_time', not getattr(self, '_video_sort_time', True)), self._refresh_video_list())); top.addWidget(self.btn_sort_video); vl.addLayout(top)
        self.video_list = QListWidget(); self.video_list.setStyleSheet("font-size:16px; QListWidget::indicator { width: 24px; height: 24px; }"); self.video_list.setSelectionMode(QAbstractItemView.NoSelection); self.video_list.itemDoubleClicked.connect(self._play_video); vl.addWidget(self.video_list)
        btns = QHBoxLayout(); self.btn_delete_videos = QPushButton("删除所选"); self.btn_delete_videos.setMinimumHeight(40); self.btn_delete_videos.setStyleSheet("background-color:#8b0000; color:white; font-size:18px;"); self.btn_delete_videos.clicked.connect(self._delete_selected_videos); btns.addWidget(self.btn_delete_videos)
        bb2 = QPushButton("← 返回"); bb2.setMinimumHeight(40); bb2.clicked.connect(self.show_cache_page); btns.addWidget(bb2); vl.addLayout(btns); self.stack_layout.addWidget(self.video_widget)

        self.speed_menu_widget = self._page("速度显示", [("仅显示速度 >", self.show_gauge_only), ("记录并显示 >", self.show_gauge_record), ("波形显示 >", self.show_waveform), ("← 返回设置", self.show_settings)]); self.stack_layout.addWidget(self.speed_menu_widget)

        self.gauge_widget = QWidget(); gl = QVBoxLayout(); self.gauge_widget.setLayout(gl)
        gt = QLabel("速度仪表盘"); gt.setAlignment(Qt.AlignCenter); gt.setStyleSheet("font-size:28px; font-weight:bold;"); gl.addWidget(gt)
        gr = QHBoxLayout(); self.gauge_left = SpeedGauge("前向 Vx", 1.0); self.gauge_right = SpeedGauge("旋转 Vz", 1.0); gr.addWidget(self.gauge_left); gr.addWidget(self.gauge_right); gl.addLayout(gr)
        gbb = QPushButton("← 返回"); gbb.setMinimumHeight(40); gbb.clicked.connect(self.show_speed_menu); gl.addWidget(gbb); self.stack_layout.addWidget(self.gauge_widget)

        self.gauge_rec_widget = QWidget(); rl = QVBoxLayout(); rl.setContentsMargins(0,0,0,0); rl.setSpacing(0); self.gauge_rec_widget.setLayout(rl)
        rr = QHBoxLayout(); rr.setSpacing(0); rr.setContentsMargins(0,0,0,0)
        self.rec_camera_container = QWidget(); self.rec_camera_container.setLayout(QVBoxLayout());
        self.rec_camera_container.layout().setContentsMargins(0,0,0,0); self.rec_camera_container.layout().setSpacing(0)
        rr.addWidget(self.rec_camera_container, 2)
        # 右边: 上=微型波形, 下=文字数据
        rv = QVBoxLayout(); rv.setSpacing(0); rv.setContentsMargins(0,0,0,0)
#        import pyqtgraph as pg  # lazy import
        self._rec_plot = pg.PlotWidget(); self._rec_plot.setBackground('#14161b'); self._rec_plot.showGrid(x=True, y=True, alpha=0.3)
        self._rec_plot.setLabel('left', '')
        self._rec_curve_vx = self._rec_plot.plot(pen=pg.mkPen('#4ade80', width=2), name='Vx')
        self._rec_curve_vz = self._rec_plot.plot(pen=pg.mkPen('#60a5fa', width=2), name='Vz')
        self._rec_curve_b  = self._rec_plot.plot(pen=pg.mkPen('#facc15', width=1), name='Bat')
        rv.addWidget(self._rec_plot, 1)
        self.label_rec_data = QLabel("<span style='color:#4ade80'>● 前向</span>  <span style='color:#60a5fa'>● 旋转</span>  <span style='color:#facc15'>● 电池</span>"); self.label_rec_data.setStyleSheet("font-size:14px; font-weight:bold; padding:2px;"); self.label_rec_data.setAlignment(Qt.AlignCenter); self.label_rec_data.setTextFormat(Qt.RichText); self.label_rec_data.setMaximumHeight(22)
        rv.addWidget(self.label_rec_data, 0)
        self.label_rec_values = QLabel("Vx: --  Vz: --  电池: --"); self.label_rec_values.setStyleSheet("font-size:18px; font-weight:bold; padding:4px;"); self.label_rec_values.setAlignment(Qt.AlignCenter); self.label_rec_values.setTextFormat(Qt.RichText)
        rv.addWidget(self.label_rec_values, 0)
        rv.addStretch(1)
        rr.addLayout(rv, 1)
        rl.addLayout(rr, 1)
        rbb = QPushButton("← 返回"); rbb.setMaximumHeight(36); rbb.setMinimumHeight(36); rbb.clicked.connect(self.show_speed_menu); rl.addWidget(rbb); self.stack_layout.addWidget(self.gauge_rec_widget)

        self.waveform_widget = QWidget(); wl = QVBoxLayout(); self.waveform_widget.setLayout(wl)
        from ui.waveform_widget import WaveformWidget
        self._waveform = WaveformWidget(); wl.addWidget(self._waveform)
        wbb = QPushButton("← 返回"); wbb.setMinimumHeight(40); wbb.clicked.connect(self.show_speed_menu); wl.addWidget(wbb)
        self.stack_layout.addWidget(self.waveform_widget)
        self.slam_widget = QWidget(); sl2 = QVBoxLayout(); self.slam_widget.setLayout(sl2)
        st2 = QLabel("SLAM 建图"); st2.setAlignment(Qt.AlignCenter); st2.setStyleSheet("font-size:28px;font-weight:bold;")
        sl2.addWidget(st2); sl2.addSpacing(10)
        self._btn_start_slam = QPushButton("开始建图"); self._btn_start_slam.setMinimumHeight(45)
        self._btn_start_slam.setStyleSheet("font-size:20px;background-color:#1e6b3b;color:white;")
        self._btn_start_slam.clicked.connect(self.start_mapping); sl2.addWidget(self._btn_start_slam); sl2.addSpacing(8)
        self._btn_save_map = QPushButton("保存地图"); self._btn_save_map.setMinimumHeight(45)
        self._btn_save_map.setStyleSheet("font-size:20px;"); self._btn_save_map.clicked.connect(self._save_map); sl2.addWidget(self._btn_save_map)
        sl2.addSpacing(8)
        self._btn_nav = QPushButton("Nav2 导航"); self._btn_nav.setMinimumHeight(45)
        self._btn_nav.setStyleSheet("font-size:20px;background-color:#e67e22;color:white;")
        self._btn_nav.clicked.connect(self.start_nav); sl2.addWidget(self._btn_nav)
        sl2.addSpacing(8)
        self._btn_map_gallery = QPushButton("地图库"); self._btn_map_gallery.setMinimumHeight(45)
        self._btn_map_gallery.setStyleSheet("font-size:20px;background-color:#1565c0;color:white;")
        self._btn_map_gallery.clicked.connect(self.show_map_gallery); sl2.addWidget(self._btn_map_gallery)
        self._map_container = QVBoxLayout()
        sl2.addLayout(self._map_container, stretch=1)
        sl2.addStretch()
        bb2 = QPushButton("返回主界面"); bb2.setMinimumHeight(40); bb2.clicked.connect(self.show_main); sl2.addWidget(bb2)
        self.stack_layout.addWidget(self.slam_widget)
        self.map_gallery_widget = QWidget(); self.map_gallery_widget.setLayout(QVBoxLayout())
        self.stack_layout.addWidget(self.map_gallery_widget)

    def _page(self, title, buttons):
        w = QWidget(); l = QVBoxLayout(); w.setLayout(l)
        t = QLabel(title); t.setAlignment(Qt.AlignCenter); t.setStyleSheet("font-size:32px; font-weight:bold;"); l.addWidget(t); l.addSpacing(15)
        for text, handler in buttons:
            btn = QPushButton(text); btn.setMinimumHeight(42); btn.setStyleSheet("font-size:20px;"); btn.clicked.connect(handler); l.addWidget(btn)
            if text != buttons[-1][0]: l.addSpacing(6)
        return w

    def _refresh_video_list(self):
        self.video_list.clear()
        os.makedirs(VIDEO_DIR, exist_ok=True)
        all_files = glob.glob(os.path.join(VIDEO_DIR, "*.avi")) + glob.glob(os.path.join(VIDEO_DIR, "*.mp4"))
        # 过滤空壳文件（< 100KB 就是摄像头没开时录的）
        all_files = [f for f in all_files if os.path.getsize(f) > 100 * 1024]
        if getattr(self, '_video_sort_time', True):
            files = sorted(all_files, key=lambda f: os.path.getmtime(f), reverse=True)
            self.btn_sort_video.setText("时间▼")
        else:
            files = sorted(all_files, reverse=True)
            self.btn_sort_video.setText("名称▼")
        self.video_count_label.setText(f"共 {len(files)} 个视频")
        for f in files:
            sz = os.path.getsize(f) / 1048576.0
            ts = datetime.datetime.fromtimestamp(os.path.getmtime(f)).strftime("%Y-%m-%d %H:%M:%S")
            item = QListWidgetItem(f"{os.path.basename(f)}    {sz:.1f}MB    {ts}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable); item.setCheckState(Qt.Unchecked); item.setData(Qt.UserRole, f)
            self.video_list.addItem(item)

    def _play_video(self, item):
        """双击播放视频 — OpenCV 读 MJPG/AVI（Docker 兼容）"""
        path = item.data(Qt.UserRole)
        if not path or not os.path.exists(path):
            return
        import cv2
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
        from PyQt5.QtGui import QImage, QPixmap

        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            self.status_label.setText(f"状态:\n视频:\n{os.path.basename(path)}")
            self.log_send("视频文件: " + path)
            return

        fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 960)
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)

        dlg = QDialog(self); dlg.setWindowTitle(os.path.basename(path))
        dlg.resize(min(w, 960), min(h, 540))
        lbl = QLabel(); lbl.setAlignment(Qt.AlignCenter)
        lay = QVBoxLayout(); lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(lbl); dlg.setLayout(lay)

        timer = QTimer()
        def _next():
            ret, frame = cap.read()
            if not ret:
                cap.release(); timer.stop(); return
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hf, wf = frame.shape[:2]
            qimg = QImage(frame.data, wf, hf, wf * 3, QImage.Format_RGB888)
            lbl.setPixmap(QPixmap.fromImage(qimg).scaled(
                lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        timer.timeout.connect(_next); timer.start(int(1000 / fps))
        dlg.finished.connect(lambda: (cap.release(), timer.stop()))
        dlg.exec_()

    def _toggle_select_all(self):
        checked = self.btn_select_all.text() == "全选"
        state = Qt.Checked if checked else Qt.Unchecked
        for i in range(self.video_list.count()): self.video_list.item(i).setCheckState(state)
        self.btn_select_all.setText("取消全选" if checked else "全选")

    def _delete_selected_videos(self):
        to_del = [self.video_list.item(i).data(Qt.UserRole) for i in range(self.video_list.count()) if self.video_list.item(i).checkState() == Qt.Checked]
        if not to_del: QMessageBox.information(self, "提示", "未选择任何视频"); return
        if QMessageBox.question(self, "确认删除", f"确定删除 {len(to_del)} 个视频？", QMessageBox.Yes|QMessageBox.No) == QMessageBox.Yes:
            for f in to_del:
                try: os.remove(f)
                except: pass
            self._refresh_video_list(); self.btn_select_all.setText("全选"); self.log_send(f"删除 {len(to_del)} 个视频")

    def _clear_config(self):
        if QMessageBox.question(self, "确认", "确定清除所有配置？", QMessageBox.Yes|QMessageBox.No) == QMessageBox.Yes:
            if os.path.exists(CONFIG_FILE): os.remove(CONFIG_FILE)
            self.config = {"hw_mode":0, "servo":True, "ultrasonic":False}; self._save_config()
            QMessageBox.information(self, "完成", "配置已清除"); self.log_send("配置已清除")

    def _update_speed(self):
        vl = self._speed_left if abs(self._speed_left) > 0.001 else 0.0
        vr = self._speed_right if abs(self._speed_right) > 0.001 else 0.0
        if hasattr(self, 'gauge_left'): self.gauge_left.set_value(vl); self.gauge_right.set_value(vr)
        try:
            b = self._rec_b_buf[-1] if self._rec_b_buf else 0
            self.label_rec_values.setText(
                f"<span style='color:#4ade80'>Vx:{vl:.2f}</span>  "
                f"<span style='color:#60a5fa'>Vz:{vr:.2f}</span>  "
                f"<span style='color:#facc15'>电池:{b:.1f}V</span>")
        except Exception:
            pass

    def _save_hw(self):
        from core.hw_detector import has_usb_camera, has_depth_camera, has_lidar
        mode = self.hw_group.checkedId()
        if mode < 0: mode = 0
        missing = []
        if mode == 0 and not has_usb_camera(): missing.append("USB 摄像头")
        elif mode == 1:
            if not has_depth_camera(): missing.append("深度相机")
            if not has_lidar(): missing.append("激光雷达")
        if missing and mode != 2:
            cur = self._detect_mode_name(2)
            QMessageBox.warning(self, "硬件检测警告", f"未检测到: {', '.join(missing)}\n\n已自动切换到「自动识别」模式\n当前模式: {cur}")
            mode = 2; self.rb_auto.setChecked(True)
        else:
            from core.hw_detector import get_device_names
            devs = get_device_names()
            devs_text = "\n".join([f"  · {d}" for d in devs])
            if "未识别" in devs[0]:
                QMessageBox.warning(self, "硬件检测",
                    f"未识别设备，请检查连接\n\n当前模式: {self._detect_mode_name(mode)}")
            else:
                QMessageBox.information(self, "保存成功",
                    f"已检测到设备:\n{devs_text}\n\n当前模式: {self._detect_mode_name(mode)}")
        self.config["hw_mode"] = mode; self._save_config(); self.hw_save_hint.setText(f"已保存 ({self._detect_mode_name()})")

    def _detect_mode_name(self, mode=None):
        from core.hw_detector import has_depth_camera, has_lidar
        if mode is None: mode = self.config.get("hw_mode", 0)
        if mode == 0: return "摄像头(普通)"
        elif mode == 1:
            parts = []; has_depth_camera() and parts.append("深度相机"); has_lidar() and parts.append("雷达")
            return "双目+雷达: " + (" + ".join(parts) if parts else "未检测到")
        elif mode == 2:
            from core.hw_detector import auto_detect; names = {0:"普通", 1:"深度+雷达", -1:"无设备"}; return f"自动识别: {names.get(auto_detect(), '未知')}"
        return "未知"

    def clear_send_log(self): self.send_log.clear()
    def clear_recv_log(self): self.recv_log.clear()
    def log_send(self, label=""): self.send_log.append(f"[{label}]"); self.send_log.moveCursor(QTextCursor.End)
    def log_recv(self, text): self.recv_log.append(text); self.recv_log.moveCursor(QTextCursor.End)

    def _release_camera(self):
        self._stop_grab_record()
        if self.camera_widget: self.camera_widget.stop_recording(); self.camera_widget.stop_camera(); self.camera_widget = None
        if self.camera_window: self.camera_window.close(); self.camera_window = None

    def _on_camera_window_close(self, event): self._release_camera(); event.accept()
    def closeEvent(self, event):
        self._abort_prepare()
        if TEST_MODE and getattr(self, '_mapping', False):
            self._stop_mock_mapping()
        self._pending_stop_after_save = False; self._saving = False
        self._release_camera()
        if hasattr(self, 'rec_camera') and self.rec_camera: self.rec_camera.stop_recording(); self.rec_camera.stop_camera()
        import subprocess
        subprocess.run(
            ["bash", "-c",
             self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' >/dev/null 2>&1"],
            env=dict(os.environ, DISPLAY=":0"))

        # 先优雅结束本进程亲手拉起的子进程，再交给 pkill 兜底（句柄自己维护的不靠模式匹配）
        for _attr in ("_lidar_proc", "_bringup_proc", "_bridge_proc"):
            _p = getattr(self, _attr, None)
            try:
                if _p is not None and _p.poll() is None:
                    _p.terminate()
            except Exception:
                pass
        # v3 分层清理：叠加层（slam / Nav2 / rviz2 / 看图 map_server）→ 底层组（bringup/EKF/静态 TF/雷达）→ 桥
        # 1. 叠加层
        subprocess.run(["pkill", "-f",
                        "async_slam_toolbox_node|sync_slam_toolbox_node|mapper_online_async.launch.py|mapper_online_sync.launch.py|"
                        "bringup_launch.py|nav2_container|component_container_isolated|"
                        "controller_server|planner_server|bt_navigator|behavior_server|"
                        "smoother_server|waypoint_follower|amcl|map_server|lifecycle_manager_navigation|"
                        "rviz2"])
        # 2. 底层组（EKF + 静态 TF + 雷达预留）
        subprocess.run(["pkill", "-f",
                        "mower_bringup.launch.py|ekf_filter_node|ekf_node|"
                        "static_transform_publisher|base_to_|rplidar_node"])
        # 3. 底盘桥
        subprocess.run(["pkill", "-f", "chassis_bridge.launch.py|chassis_bridge"])
        # 刷新陈旧节点图缓存
        subprocess.run(["bash", "-c", self.ROS_SETUP + " && ros2 daemon stop >/dev/null 2>&1"],
                       env=dict(os.environ, DISPLAY=":0"))
        self.serial_mgr.close(); event.accept()

    # ============ 录屏（预览窗 + OpenCV 录像，Docker 兼容） ============
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
            self.serial_mgr.send(self.proto.build_emergency_frame())

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

    def _start_bridge(self):
        """v3：底盘桥启动即拉、常驻在线；叠加层启停不重启本层。Mock 模式跳过。"""
        import subprocess, os
        self._bridge_proc = subprocess.Popen(
            ["bash", "-c", self.BRIDGE_SETUP + " && " + self.BRIDGE_LAUNCH],
            env=dict(os.environ, DISPLAY=":0"))

    # ================= 非阻塞准备流程（QTimer 状态机） =================
    # 拉起底层组/雷达要等就绪，但不能在 GUI 线程里 sleep 轮询（窗口会假死）。
    # 这里改成每 500 ms 推进一次的状态机：界面始终可响应，超时明确提示。
    def _topic_online(self, topic):
        """话题是否有发布者。优先问本地 rclpy 节点（O(1)），失败再回退到 ros2 CLI。"""
        fn = getattr(self.serial_mgr, "topic_online", None)
        if callable(fn):
            try:
                return bool(fn(topic))
            except Exception:
                pass
        import subprocess, os
        probe = self.ROS_SETUP + " && ros2 topic list 2>/dev/null | grep -x " + topic
        try:
            return subprocess.run(["bash", "-c", probe], env=dict(os.environ, DISPLAY=":0"),
                                  capture_output=True, text=True, timeout=5).returncode == 0
        except Exception:
            return False

    def _ensure_bridge(self):
        """确保底盘桥在跑：/odom 在线直接返回；已拉起且进程活着就等；否则重新拉起。

        桥只在 UI 启动时拉一次，运行中若挂了（串口松动、进程被杀），
        后续建图会一直卡在等 /odom_combined。这里补上"桥死了就重拉"。
        """
        if TEST_MODE or self._topic_online("/odom"):
            return
        p = getattr(self, "_bridge_proc", None)
        if p is not None and p.poll() is None:
            return          # 进程还活着，可能只是还没开始发数据，交给超时判断
        self.log_send("底盘桥: 未在线，重新拉起")
        self._start_bridge()

    def _ensure_bringup(self):
        """只拉起、不等待：/odom_combined 已在线则跳过。Mock 跳过。

        已经拉起过且进程还活着就不再拉第二份——上一次准备超时后重试时，
        若无此判断会叠出第二套 EKF / 静态 TF / robot_state_publisher。
        """
        if TEST_MODE or self._topic_online("/odom_combined"):
            return
        p = getattr(self, "_bringup_proc", None)
        if p is not None and p.poll() is None:
            return
        import subprocess, os
        self.log_send("底层组: 启动 bringup（EKF/静态 TF/URDF）")
        self._bringup_proc = subprocess.Popen(
            ["bash", "-c", self.ROS_SETUP + " && " + self.BRINGUP_LAUNCH],
            env=dict(os.environ, DISPLAY=":0"))

    def _ensure_lidar(self):
        """只拉起、不等待：/scan 已在线则跳过。不整组带 use_lidar，避免复制第二套 EKF。"""
        if TEST_MODE or self._topic_online("/scan"):
            return
        p = getattr(self, "_lidar_proc", None)
        if p is not None and p.poll() is None:
            return          # 已拉起且还活着：再开一个会抢同一个串口
        import subprocess, os
        params = ROS_WS + "/install/mower_bringup/share/mower_bringup/config/lidar_node_params.yaml"
        self.log_send("激光雷达: 单起 rplidar_node")
        self._lidar_proc = subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 run rplidar_ros rplidar_node --ros-args --params-file " + params],
            env=dict(os.environ, DISPLAY=":0"))

    def _prepare(self, steps, done):
        """非阻塞准备：steps=[(阶段名, 就绪判定, 超时秒, 进入动作)]，done(ok) 回到 GUI 线程。

        建图与导航共用这一条流水线：若上一次还没结束就再次触发，先取消上一次
        （并恢复按钮），再把两个入口一起禁用，直到本次结束。
        """
        if getattr(self, "_prep_steps", None):
            self.log_send("准备流程: 取消上一次未完成的启动")
            self._abort_prepare()
        for name in ("_btn_start_slam", "_btn_nav"):
            b = getattr(self, name, None)
            if b is not None:
                try:
                    b.setEnabled(False)
                except Exception:
                    pass
        self._prep_steps = list(steps)
        self._prep_done = done
        self._prep_cur = None
        if getattr(self, "_prep_timer", None) is None:
            self._prep_timer = QTimer(self)
            self._prep_timer.timeout.connect(self._prep_tick)
        self._prep_timer.start(500)
        self._prep_tick()

    def _prep_tick(self):
        import time
        if not getattr(self, "_prep_steps", None):
            self._prep_timer.stop()
            cb, self._prep_done = self._prep_done, None
            if cb:
                cb(True)
            return
        name, cond, timeout, on_enter = self._prep_steps[0]
        if self._prep_cur != name:
            self._prep_cur = name
            self._prep_t0 = time.monotonic()
            self.log_send(name)
            self.status_label.setText("状态:\n" + name)
            if on_enter:
                try:
                    on_enter()
                except Exception as e:
                    self.log_send("准备失败: " + str(e))
        try:
            ready = bool(cond())
        except Exception:
            ready = False
        if ready:
            self._prep_steps.pop(0)
            self._prep_cur = None
            return
        if time.monotonic() - self._prep_t0 > timeout:
            self._prep_timer.stop()
            self._prep_cur = None
            self.log_send(name + ": 超时")
            self.status_label.setText("状态:\n" + name + " 未就绪")
            cb, self._prep_done = self._prep_done, None
            if cb:
                cb(False)

    def _restore_start_buttons(self):
        """把"开始建图 / Nav2 导航"恢复到可点。所有准备流程的出口都必须经过这里。"""
        for name in ("_btn_start_slam", "_btn_nav"):
            b = getattr(self, name, None)
            if b is not None:
                try:
                    b.setEnabled(True)
                except Exception:
                    pass

    def _abort_prepare(self):
        """取消准备流程。必须把按钮恢复，否则急停打断后"开始建图"会一直灰着。"""
        t = getattr(self, "_prep_timer", None)
        if t is not None:
            t.stop()
        self._prep_steps = []
        self._prep_done = None
        self._prep_cur = None
        self._restore_start_buttons()

    def start_mapping(self):
        if TEST_MODE:
            self._start_mock_mapping()
            return
        # --- 停止建图（叠加层；底层组与桥常驻，不重启、不恢复） ---
        if getattr(self, '_mapping', False):
            self._request_stop_mapping()
            return

        # 建图模式二选一（取消则不启动）
        mode = self._choose_slam_mode()
        if mode is None:
            return
        self._slam_mode = mode
        self.log_send("建图模式: " + ("同步" if mode == "sync" else "异步"))

        # 非阻塞准备：底层组 → 雷达，就绪后再拉叠加层（不在 GUI 线程 sleep）
        self._prepare(
            [("等待底盘桥 /odom",
              lambda: self._topic_online("/odom"), 8, self._ensure_bridge),
             ("等待底层组（EKF/静态 TF）",
              lambda: self._topic_online("/odom_combined"), 12, self._ensure_bringup),
             ("等待激光雷达 /scan",
              lambda: self._topic_online("/scan"), 8, self._ensure_lidar)],
            self._start_mapping_overlay)

    def _choose_slam_mode(self):
        """建图模式二选一：异步（默认）/ 同步。返回 'async' / 'sync'，取消返回 None。"""
        from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QRadioButton,
                                     QButtonGroup, QDialogButtonBox, QLabel)
        d = QDialog(self)
        d.setWindowTitle("选择建图模式")
        lay = QVBoxLayout(); d.setLayout(lay)
        lay.addWidget(QLabel("slam_toolbox 建图模式："))
        rb_async = QRadioButton("异步（推荐）：建图与位姿解算分开跑，不丢帧，CPU 占用略高")
        rb_sync = QRadioButton("同步：严格按帧顺序处理，慢但可复现，用于排查建图异常")
        rb_async.setChecked(True)
        g = QButtonGroup(d); g.addButton(rb_async); g.addButton(rb_sync)
        lay.addWidget(rb_async); lay.addWidget(rb_sync)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(d.accept); bb.rejected.connect(d.reject)
        lay.addWidget(bb)
        if not d.exec_():
            return None
        return "sync" if rb_sync.isChecked() else "async"

    def _start_mapping_overlay(self, ok):
        """准备完成回调（GUI 线程）：起 slam_toolbox + rviz2。"""
        self._restore_start_buttons()
        if not ok:
            self.log_send("建图: 准备未完成，已取消（查日志：底层组或雷达未就绪）")
            self.status_label.setText("状态:\n准备未完成")
            return
        import subprocess, os
        mode = getattr(self, "_slam_mode", "async")
        launch_file = ("mapper_online_sync.launch.py" if mode == "sync"
                       else "mapper_online_async.launch.py")
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 launch mower_slam " + launch_file],
            env=dict(os.environ, DISPLAY=":0"))
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && rviz2 -d " + RVIZ_CFG],
            env=dict(os.environ, DISPLAY=":0"))
        self._mapping = True
        self._map_saved = False
        self._pending_stop_after_save = False
        self._saving = False
        self._btn_start_slam.setText("停止建图")
        self.status_label.setText("状态:\n建图中(slam_toolbox-%s)" % ("同步" if mode == "sync" else "异步"))
        self.log_send("建图: slam_toolbox 已启动（%s）" % ("同步" if mode == "sync" else "异步"))

    def show_slam_page(self):
        self.setWindowTitle("SLAM 地图"); self.stack_layout.setCurrentWidget(self.slam_widget)

    def _save_map(self):
        if not getattr(self, '_mapping', False):
            self.log_send("保存地图: 当前未在建图，无地图可保存")
            self.status_label.setText("状态:\n未在建图")
            QMessageBox.warning(self, "保存地图", "请先开始建图，地图发布后再保存。")
            return
        if getattr(self, '_saving', False):
            return
        from datetime import datetime
        import subprocess, os, threading
        os.makedirs(MAP_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(MAP_DIR, "mymap_" + ts)
        self.log_send("保存地图: " + fname); self.status_label.setText("状态:\n保存地图中...")
        self._btn_save_map.setEnabled(False)
        self._btn_start_slam.setEnabled(False)
        self._saving = True

        def _run():
            try:
                r = subprocess.run(
                    ["bash", "-c",
                     self.ROS_SETUP + " && ros2 run nav2_map_server map_saver_cli -f " + fname + " --ros-args -p map_subscribe_transient_local:=false -p save_map_timeout:=10.0"],
                    env=dict(os.environ, DISPLAY=":0"), capture_output=True, text=True, timeout=35)
                raw = r.stderr.strip() or r.stdout.strip()
                lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]
                err_lines = [ln for ln in lines
                             if "[ERROR]" in ln or "[WARN]" in ln
                             or "failed" in ln.lower() or "error" in ln.lower()]
                err = " / ".join(err_lines) or (lines[-1] if lines else "")
                ok = r.returncode == 0 or os.path.exists(fname + ".pgm")
            except Exception as e:
                err = str(e); ok = False
            self.save_done.emit(ok, fname, err[:160] if err else "")

        threading.Thread(target=_run, daemon=True).start()

    def _on_save_done(self, ok, fname, err):
        self._btn_save_map.setEnabled(True)
        self._btn_start_slam.setEnabled(True)
        self._saving = False
        if ok:
            self._map_saved = True
            self.log_send("地图保存成功: " + fname); self.status_label.setText("状态:\n地图已保存")
        else:
            self.log_send("保存失败: " + (err or "未知错误"))
            self.status_label.setText("状态:\n保存失败")
            if self._pending_stop_after_save:
                box = QMessageBox(self)
                box.setWindowTitle("停止建图")
                box.setText("地图保存失败，仍要停止建图吗？")
                b_yes = box.addButton("仍要停止", QMessageBox.AcceptRole)
                box.addButton("继续建图", QMessageBox.RejectRole)
                box.exec_()
                self._pending_stop_after_save = False
                if box.clickedButton() is b_yes:
                    self._do_stop_mapping()
                else:
                    self.status_label.setText("状态:\n建图中")
                return
        if self._pending_stop_after_save:
            self._pending_stop_after_save = False
            self._do_stop_mapping()

    def _request_stop_mapping(self):
        """停止建图统一入口：本次会话未保存过先弹窗询问；保存过直接清理。"""
        if self._map_saved:
            self._do_stop_mapping()
            return
        box = QMessageBox(self)
        box.setWindowTitle("停止建图")
        box.setText("当前地图尚未保存，是否先保存再停止？")
        b_save = box.addButton("保存并停止", QMessageBox.AcceptRole)
        box.addButton("不保存，直接停止", QMessageBox.DestructiveRole)
        b_cancel = box.addButton("取消", QMessageBox.RejectRole)
        box.exec_()
        if box.clickedButton() is b_cancel:
            return
        if box.clickedButton() is b_save:
            self._pending_stop_after_save = True
            self._save_map()
            return
        self._do_stop_mapping()

    def _do_stop_mapping(self):
        """执行实际停止清理：Mock/真机分发，动作口径对齐各自手动停止分支。"""
        if TEST_MODE:
            self._stop_mock_mapping()
        else:
            self._stop_mapping_real()

    def _stop_mapping_real(self):
        """真机停止建图清理（动作平移自 start_mapping 原停止分支）。"""
        import subprocess
        self._mapping = False
        self._btn_start_slam.setText("开始建图")
        self.status_label.setText("状态:\n就绪")
        self.log_send("建图: 已停止")
        subprocess.run(
            ["bash", "-c",
             self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' >/dev/null 2>&1"],
            env=dict(os.environ, DISPLAY=":0"))
        subprocess.run(["pkill", "-f", "async_slam_toolbox_node|sync_slam_toolbox_node|mapper_online_async.launch.py|mapper_online_sync.launch.py"])
        subprocess.run(["pkill", "rviz2"])

    def start_nav(self):
        """Nav2 导航 — 点一下启动选地图，再点停止恢复控制"""
        if TEST_MODE:
            self.log_send("导航: Mock 模式不支持 Nav2 导航")
            QMessageBox.information(self, "Mock 模式", "当前为 Mock 环境（无硬件），Nav2 导航请在真机模式使用。")
            return
        import subprocess, os, glob

        # --- 停止导航（叠加层；底层组与桥常驻，不重启、不恢复） ---
        if getattr(self, '_nav_on', False):
            self._nav_on = False
            self._btn_nav.setText("Nav2 导航")
            self.status_label.setText("状态:\n就绪")
            self.log_send("导航: 停止")
            # 杀 Nav2 叠加层：launch 主进程 + composition 容器 + 独立节点兜底
            subprocess.run(["pkill", "-f",
                            "bringup_launch.py|nav2_container|component_container_isolated|"
                            "controller_server|planner_server|bt_navigator|behavior_server|"
                            "smoother_server|waypoint_follower|amcl|map_server|lifecycle_manager_navigation"])
            subprocess.run(["pkill", "rviz2"])
            subprocess.run(
                ["bash", "-c",
                 self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' >/dev/null 2>&1"],
                env=dict(os.environ, DISPLAY=":0"))
            return

        # --- 启动导航 ---
        maps = sorted(glob.glob(os.path.join(MAP_DIR, "mymap*.yaml")), reverse=True)
        if not maps:
            self.log_send("导航: 没有已保存的地图，请先建图")
            return

        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QListWidget, QDialogButtonBox, QLabel
        dlg = QDialog(self)
        dlg.setWindowTitle("选择地图")
        dlg.setMinimumSize(400, 300)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lay.addWidget(QLabel("选择导航用的地图:"))
        lst = QListWidget()
        for m in maps:
            lst.addItem(os.path.basename(m))
        lst.setCurrentRow(0)
        lay.addWidget(lst)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return
        map_file = maps[lst.currentRow()]
        self.log_send("导航: 加载 " + os.path.basename(map_file))

        # 停建图叠加层（叠加层互斥；底层组与桥常驻不杀）
        if getattr(self, '_mapping', False):
            subprocess.run(["pkill", "-f", "async_slam_toolbox_node|sync_slam_toolbox_node|mapper_online_async.launch.py|mapper_online_sync.launch.py"])
            subprocess.run(["pkill", "rviz2"])
            self._mapping = False

        # 清独立看图残留（map_server/rviz2），避免与 Nav2 自带 map_server 双发布 /map
        subprocess.run(["pkill", "-f", "map_server"])
        subprocess.run(["pkill", "rviz2"])

        # 停建图叠加层（叠加层互斥；底层组与桥常驻不杀）
        if getattr(self, '_mapping', False):
            subprocess.run(["pkill", "-f", "async_slam_toolbox_node|sync_slam_toolbox_node|mapper_online_async.launch.py|mapper_online_sync.launch.py"])
            subprocess.run(["pkill", "rviz2"])
            self._mapping = False
            self._btn_start_slam.setText("开始建图")

        # 清独立看图残留（map_server/rviz2），避免与 Nav2 自带 map_server 双发布 /map
        subprocess.run(["pkill", "-f", "map_server"])
        subprocess.run(["pkill", "rviz2"])

        # 非阻塞准备：底层组 → 雷达，就绪后再拉 Nav2 叠加层
        self._pending_map_file = map_file
        self._prepare(
            [("等待底盘桥 /odom",
              lambda: self._topic_online("/odom"), 8, self._ensure_bridge),
             ("等待底层组（EKF/静态 TF）",
              lambda: self._topic_online("/odom_combined"), 12, self._ensure_bringup),
             ("等待激光雷达 /scan",
              lambda: self._topic_online("/scan"), 8, self._ensure_lidar)],
            self._start_nav_overlay)

    def _start_nav_overlay(self, ok):
        """准备完成回调（GUI 线程）：起 nav2，延时再起 rviz2。"""
        self._restore_start_buttons()
        if not ok:
            self.log_send("导航: 准备未完成，已取消（查日志：底层组或雷达未就绪）")
            self.status_label.setText("状态:\n准备未完成")
            return
        import subprocess, os
        map_file = getattr(self, "_pending_map_file", "")
        nav2_params = ROS_WS + "/install/mower_nav2/share/mower_nav2/config/nav2_params.yaml"
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 launch nav2_bringup bringup_launch.py"
             " map:=" + map_file + " params_file:=" + nav2_params + " use_sim_time:=false"],
            env=dict(os.environ, DISPLAY=":0"))
        self._nav_on = True
        self._btn_nav.setText("停止导航")
        self.status_label.setText("状态:\n导航中")
        self.log_send("导航: 已启动，在 rviz2 用 2D Goal Pose 点目标")
        # 原来 time.sleep(3) 会卡主线程，改成单次定时器
        QTimer.singleShot(3000, self._start_nav_rviz)

    def _start_nav_rviz(self):
        if not getattr(self, "_nav_on", False):
            return
        import subprocess, os
        subprocess.Popen(
            ["bash", "-c", self.ROS_SETUP + " && rviz2 -d " + RVIZ_CFG],
            env=dict(os.environ, DISPLAY=":0"))

    def show_map_gallery(self):
        self.setWindowTitle("地图库"); self._refresh_gallery(); self.stack_layout.setCurrentWidget(self.map_gallery_widget)

    def _refresh_gallery(self):
        import glob, os
        lay = self.map_gallery_widget.layout()
        # 首次创建完整布局
        if not getattr(self, '_gallery_init', False):
            self._gallery_init = True
            # Header
            hdr = QHBoxLayout()
            t = QLabel("地图库"); t.setStyleSheet("font-size:28px;font-weight:bold;")
            hdr.addWidget(t); hdr.addStretch()
            btn_select = QPushButton("全选"); btn_select.setMinimumHeight(35)
            btn_select.clicked.connect(lambda: (
                [self._map_list.item(i).setCheckState(
                    Qt.Unchecked if btn_select.text() == "取消全选" else Qt.Checked)
                 for i in range(self._map_list.count())],
                btn_select.setText("全选" if btn_select.text() == "取消全选" else "取消全选")
            ))
            hdr.addWidget(btn_select)
            btn_refresh = QPushButton("刷新"); btn_refresh.clicked.connect(self._refresh_gallery)
            hdr.addWidget(btn_refresh)
            btn_back = QPushButton("返回"); btn_back.clicked.connect(self.show_slam_page)
            hdr.addWidget(btn_back)
            lay.addLayout(hdr)
            # QListWidget
            self._map_list = QListWidget()
            self._map_list.setStyleSheet("font-size:16px; QListWidget::indicator { width: 24px; height: 24px; }")
            self._map_list.setSelectionMode(QAbstractItemView.NoSelection)
            self._map_list.itemDoubleClicked.connect(self._map_card_click)
            lay.addWidget(self._map_list, 1)
            # 底部删除按钮
            btn_del = QPushButton("删除所选"); btn_del.setMinimumHeight(45)
            btn_del.setStyleSheet("background-color:#8b0000; color:white; font-size:20px;")
            btn_del.clicked.connect(self._delete_selected_maps)
            lay.addWidget(btn_del)
        else:
            self._map_list.clear()

        files = sorted(glob.glob(os.path.join(MAP_DIR, "mymap*.yaml")), reverse=True)
        for yf in files:
            base = yf.replace(".yaml", ""); pgm = base + ".pgm"
            if not os.path.exists(pgm): continue
            name = os.path.basename(base); sz = os.path.getsize(pgm)
            item = QListWidgetItem(f"{name}    {sz/1024:.0f}KB")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            item.setData(Qt.UserRole, name)
            self._map_list.addItem(item)

    def _map_card_click(self, item):
        """双击地图 → 弹出操作菜单"""
        name = item.data(Qt.UserRole)
        self._card_click_dialog(name)

    def _delete_selected_maps(self):
        """删除所有勾选的地图"""
        to_del = [self._map_list.item(i).data(Qt.UserRole)
                  for i in range(self._map_list.count())
                  if self._map_list.item(i).checkState() == Qt.Checked]
        if not to_del:
            QMessageBox.information(self, "提示", "未选择任何地图")
            return
        if QMessageBox.question(self, "确认删除",
                                f"确定删除 {len(to_del)} 个地图？",
                                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        import os
        for name in to_del:
            for ext in [".pgm", ".yaml", ".ppm"]:
                p = os.path.join(MAP_DIR, name + ext)
                if os.path.exists(p):
                    os.remove(p)
        self._refresh_gallery()
        self.log_send(f"已删除 {len(to_del)} 个地图")

    def _card_click_dialog(self, name):
        """地图操作弹窗"""
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QPushButton
        dlg = QDialog(self); dlg.setWindowTitle(f"打开地图: {name}")
        dlg.setMinimumSize(320, 250)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        btn_r = QPushButton("rviz2 查看"); btn_r.setMinimumHeight(40)
        btn_r.clicked.connect(lambda: (self._open_map_rviz(name), dlg.accept()))
        lay.addWidget(btn_r)
        btn_p = QPushButton("图片查看"); btn_p.setMinimumHeight(40)
        btn_p.clicked.connect(lambda: (self._open_map_ppm(name), dlg.accept()))
        lay.addWidget(btn_p)
        btn_rn = QPushButton("重命名"); btn_rn.setMinimumHeight(40)
        btn_rn.clicked.connect(lambda: self._rename_map(name))
        lay.addWidget(btn_rn)
        btn_d = QPushButton("删除"); btn_d.setMinimumHeight(40)
        btn_d.setStyleSheet("color:red;")
        btn_d.clicked.connect(lambda: self._delete_map(name))
        lay.addWidget(btn_d)
        dlg.exec_()

    def _open_map_rviz(self, name):
        import subprocess, os
        if TEST_MODE:
            QMessageBox.information(self, "Mock 模式", "当前为 Mock 环境，独立地图查看请在真机模式使用。")
            return
        # 导航叠加层运行中不进入独立看图，避免双 map_server 发布 /map 冲突
        if getattr(self, '_nav_on', False):
            QMessageBox.warning(self, "提示", "导航进行中，请先停止导航再查看地图")
            return
        yaml_file = os.path.join(MAP_DIR, name + ".yaml")
        subprocess.run(["pkill", "-f", "map_server"])
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 run nav2_map_server map_server "
             "--ros-args -p use_sim_time:=false -p yaml_filename:=" + yaml_file],
            env=dict(os.environ, DISPLAY=":0"))
        # 等 map_server 起来再激活（原来 time.sleep(2) 会卡主线程）
        QTimer.singleShot(2000, lambda: self._activate_map_server(name))

    def _activate_map_server(self, name):
        """map_server 起来后激活 lifecycle 并开 rviz2（GUI 线程，非阻塞链的一环）。"""
        import subprocess, os
        subprocess.run(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 lifecycle set /map_server configure 2>/dev/null"
             " && ros2 lifecycle set /map_server activate 2>/dev/null"],
            env=dict(os.environ, DISPLAY=":0"))
        subprocess.Popen(
            ["bash", "-c", self.ROS_SETUP + " && rviz2 -d " + RVIZ_CFG],
            env=dict(os.environ, DISPLAY=":0"))
        self.log_send("rviz2查看(已加载map_server): " + name)

    def _open_map_ppm(self, name):
        """用 Qt 弹窗显示地图图片，不依赖外部看图软件"""
        import numpy as np
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
        from PyQt5.QtGui import QPixmap, QImage
        from PyQt5.QtCore import Qt

        pgm = os.path.join(MAP_DIR, name + ".pgm")

        # 解析 PGM 头: P5\nW H\n255\n
        with open(pgm, "rb") as f:
            header_lines = []
            for _ in range(10):
                line = f.readline()
                if not line.startswith(b'#'):
                    header_lines.append(line.strip())
                if len(header_lines) >= 3:
                    break
            raw = f.read()
        w, h = map(int, header_lines[1].split())
        raw = raw[:w * h]

        # 转 RGB 数组: 0=黑(障碍), 1~100=白(空地), 其余=灰(未知)
        img = np.zeros((h, w, 3), dtype=np.uint8)
        for i, b in enumerate(raw):
            y, x = divmod(i, w)
            if b == 0:         img[y, x] = [0, 0, 0]
            elif b <= 100:     img[y, x] = [255, 255, 255]
            else:              img[y, x] = [80, 80, 80]

        qimg = QImage(img.data, w, h, w * 3, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        dlg = QDialog(self)
        dlg.setWindowTitle(f"地图: {name}")
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lbl = QLabel()
        lbl.setPixmap(pixmap.scaled(900, 700, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        lay.addWidget(lbl)
        dlg.exec_()
        self.log_send("打开图片: " + name)


    def _rename_map(self, old):
        from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QComboBox,
                                      QDialogButtonBox, QLabel)
        dlg = QDialog(self)
        dlg.setWindowTitle("重命名")
        dlg.setMinimumWidth(350)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lay.addWidget(QLabel("选择或输入新名称:"))
        combo = QComboBox(); combo.setEditable(True); combo.setInsertPolicy(QComboBox.NoInsert)
        # 常用中文名
        presets = [
            old,  # 原名排第一
            "客厅", "卧室", "阳台", "厨房", "院子", "走廊",
            "一楼", "二楼", "三楼", "前院", "后院", "花园",
            "办公室", "仓库", "车库", "草坪", "测试区",
        ]
        for name in presets:
            combo.addItem(name)
        combo.setCurrentText(old)
        lay.addWidget(combo)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return
        new = combo.currentText().strip()
        if new and new != old:
            import os
            for ext in [".pgm", ".yaml", ".ppm"]:
                a = os.path.join(MAP_DIR, old + ext); b = os.path.join(MAP_DIR, new + ext)
                if os.path.exists(a): os.rename(a, b)
            self._refresh_gallery(); self.log_send("重命名 " + old + " -> " + new)

    def _delete_map(self, name):
        from PyQt5.QtWidgets import QMessageBox
        if QMessageBox.question(self, "确认删除", "删除地图 " + name + " ？", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            import os
            for ext in [".pgm", ".yaml", ".ppm"]:
                p = os.path.join(MAP_DIR, name + ext)
                if os.path.exists(p): os.remove(p)
            self._refresh_gallery(); self.log_send("已删除: " + name)

    def _cancel_tasks(self, for_task=""):
        """互斥: 取消当前所有运行中任务, 返回被停止的列表"""
        stopped = []
        if getattr(self, '_selftest_running', False):
            self._selftest_running = False; stopped.append("自检")
            if hasattr(self, '_selftest_timer') and self._selftest_timer: self._selftest_timer.stop()
        if getattr(self, '_dock_running', False) or (self.dock.state != "IDLE"):
            self._dock_running = False; self.dock.stop(); stopped.append("回充")
        if stopped:
            self.serial_mgr.send(self.proto.build_emergency_frame())
            self._release_camera()
            self.log_send(f"接收到新任务, 已停止 {', '.join(stopped)}")
        return stopped

    def update_leds(self, msg):
        """根据状态消息更新指示灯"""
        if '底盘桥' in msg:
            self.led_chassis.setStyleSheet("color: #00cc44; font-size: 14px;")
        elif '激光雷达' in msg:
            self.led_lidar.setStyleSheet("color: #00cc44; font-size: 14px;")
        elif 'EKF' in msg or 'odom_combined' in msg:
            self.led_ekf.setStyleSheet("color: #00cc44; font-size: 14px;")
        elif 'SLAM' in msg or '建图' in msg or '/map' in msg:
            self.led_slam.setStyleSheet("color: #00cc44; font-size: 14px;")
        elif '路径规划' in msg or 'Nav2' in msg:
            self.led_nav.setStyleSheet("color: #00cc44; font-size: 14px;")

    def release_estop(self):
        self.serial_mgr.send(self.proto.build_release_frame())
        self.status_label.setText("状态:\n急停已解除")
        self.log_send("解除急停")
        # 恢复所有按钮
        self.btn_selftest.setEnabled(True)
        self.btn_mow.setEnabled(True)
        self.btn_follow.setEnabled(True)
        self.btn_dock.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.btn_release.setEnabled(False)
        self.btn_record.setEnabled(True)

    def start_self_test(self):
        self._cancel_tasks("自检"); self._release_camera()
        self.status_label.setText("状态:\n自检中..."); self.log_send("自检: 开始")
        self._start_grab_record("自检")
        if hasattr(self, '_rec_cam_widget') and self._rec_cam_widget:
            self._rec_cam_widget.detect_green = False
        self._selftest_running = True; self.serial_mgr.send(self.proto.build_selftest_frame())
        self._selftest_timer = QTimer(); self._selftest_timer.setSingleShot(True); self._selftest_timer.timeout.connect(self._selftest_done); self._selftest_timer.start(12000)

    def _selftest_done(self):
        if not getattr(self, '_selftest_running', False): return
        self._selftest_timer.stop()
        self._selftest_running = False; self.log_send("自检: 完成"); self.status_label.setText("状态:\n就绪"); self._release_camera()

    def start_mow(self):
        # 弹窗选地图（仅展示，不加载）
        import glob, os
        maps = sorted(glob.glob(os.path.join(MAP_DIR, "mymap*.yaml")), reverse=True)
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QDialogButtonBox, QListWidget
        dlg = QDialog(self); dlg.setWindowTitle("割草区域(Nav2导航)")
        dlg.setMinimumSize(400, 280)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lay.addWidget(QLabel("选择割草地图:"))
        lst = QListWidget()
        lst.addItem("跳过 (不选地图)")
        for m in maps:
            lst.addItem(os.path.basename(m))
        lst.setCurrentRow(0); lay.addWidget(lst)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return
        if lst.currentRow() > 0:
            self.log_send("割草区域: " + os.path.basename(maps[lst.currentRow() - 1]))

        self._cancel_tasks("割草"); self._release_camera()
        # 这条路径不发刀片指令 —— 刀片控制已整套移除。
        self.status_label.setText("状态:\n割草中(无刀片控制)")
        self.log_send("割草: 开始 —— 刀片未接入, 只有录像和导航")
        self._start_grab_record("割草")
        if hasattr(self, '_rec_cam_widget') and self._rec_cam_widget:
            self._rec_cam_widget.detect_green = True

    def toggle_follow(self):
        if self._follow_on:
            self._follow_on = False
            self.btn_follow.setText("雷达跟随")
            self.status_label.setText("状态:\n就绪")
            self.log_send("雷达跟随: 已停止")
        else:
            self._follow_on = True
            self.btn_follow.setText("停止跟随")
            self.status_label.setText("状态:\n雷达启动中...")
            self.log_send("雷达跟随: 启动中...")
            QTimer.singleShot(2000, lambda: (
                self.status_label.setText("状态:\n跟随中"),
                self.log_send("雷达跟随: 已开启")
            ))

    def start_dock(self):
        self._cancel_tasks("回充"); self._release_camera()
        self.status_label.setText("状态:\n扫描中..."); self.log_send("自动回充: 开始")
        self._start_grab_record("回充")
        self._dock_running = True; self.dock.start()

    def _on_tag_found(self): self.status_label.setText("状态:\n对准中...")
    def _on_dock_done(self): self._dock_running = False; self.status_label.setText("状态:\n对接完成")

    def stop_all(self):
        self._abort_prepare()      # 取消可能还在跑的"等底层组/等雷达"流程
        stopped = []
        if getattr(self, '_selftest_running', False):
            self._selftest_running = False; stopped.append("自检")
            if hasattr(self, '_selftest_timer') and self._selftest_timer: self._selftest_timer.stop()
        if getattr(self, '_dock_running', False) or self.dock.state != "IDLE":
            self._dock_running = False; self.dock.stop(); stopped.append("回充")
        if self.camera_widget and self.camera_widget.is_recording():
            self.camera_widget.stop_recording(); stopped.append("录像")
        if self._follow_on:
            self._follow_on = False
            self.btn_follow.setText("雷达跟随"); stopped.append("跟随")
        if getattr(self, '_mapping', False):
            self._pending_stop_after_save = False; self._map_saved = False; self._saving = False
            self._btn_start_slam.setEnabled(True); self._btn_save_map.setEnabled(True)
            if TEST_MODE:
                self._stop_mock_mapping()
            else:
                self._btn_start_slam.setText("开始建图")
                # 杀 slam 叠加层：launch 主进程 + 建图节点兜底（口径同 start_mapping 停止分支）
                import subprocess, os
                subprocess.run(
                    ["bash", "-c",
                     self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' >/dev/null 2>&1"],
                    env=dict(os.environ, DISPLAY=":0"))
                subprocess.run(["pkill", "-f", "async_slam_toolbox_node|sync_slam_toolbox_node|mapper_online_async.launch.py|mapper_online_sync.launch.py"])
                subprocess.run(["pkill", "rviz2"])
                self._mapping = False; stopped.append("建图")
        if getattr(self, '_nav_on', False):
            self._nav_on = False
            self._btn_nav.setText("Nav2 导航"); stopped.append("导航")
            # 杀 Nav2 叠加层：launch 主进程 + composition 容器 + 独立节点兜底（口径同 start_nav 停止分支）
            import subprocess, os
            subprocess.run(["pkill", "-f",
                            "bringup_launch.py|nav2_container|component_container_isolated|"
                            "controller_server|planner_server|bt_navigator|behavior_server|"
                            "smoother_server|waypoint_follower|amcl|map_server|lifecycle_manager_navigation"])
            subprocess.run(["pkill", "rviz2"])
            subprocess.run(
                ["bash", "-c",
                 self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' >/dev/null 2>&1"],
                env=dict(os.environ, DISPLAY=":0"))
        self.serial_mgr.send(self.proto.build_emergency_frame())
        self._speed_left = 0; self._speed_right = 0
        self.status_label.setText("状态:\n已停止")
        self.log_send("紧急停止: " + ", ".join(stopped) if stopped else "紧急停止"); self._release_camera()
        # 锁定 UI：其他按钮禁用，只能点解除急停
        self.btn_selftest.setEnabled(False)
        self.btn_mow.setEnabled(False)
        self.btn_follow.setEnabled(False)
        self.btn_dock.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_release.setEnabled(True)
        self.btn_record.setEnabled(False)

    def _update_status(self):
        states = {'IDLE':'待机','SWEEPING':'扫描','VERIFYING':'验证','ROTATING':'旋转','UNDOCK':'退桩','DONE':'完成'}
        self.log_recv(f"[状态] 舵机:{self.dock.servo_pos}us  {states.get(self.dock.state, self.dock.state)}  "
                      f"标签:{'找到' if self.dock.tag_seen else '未检测'}")

    def open_camera_window(self):
        self.camera_window = QWidget(); self.camera_window.setWindowTitle("摄像头画面"); self.camera_window.resize(800, 600)
        layout = QVBoxLayout(); self.camera_window.setLayout(layout)
        self.camera_widget = CameraWidget(); layout.addWidget(self.camera_widget); self.camera_widget.start_camera()
        self.camera_widget.on_tag_seen = lambda: self.dock.tag_detected()
        self.camera_window.closeEvent = self._on_camera_window_close; self.camera_window.show()
