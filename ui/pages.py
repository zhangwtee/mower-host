import os
import glob
import datetime
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QLabel, QStackedLayout, QSpacerItem, QSizePolicy, QButtonGroup, QRadioButton, QMessageBox, QListWidget, QListWidgetItem, QAbstractItemView
from ui.const import CONFIG_FILE, VIDEO_DIR
from ui.speed_gauge import SpeedGauge


class PagesMixin:

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
            btn.setMinimumHeight(40); btn.setStyleSheet("font-size:20px;"); left_layout.addWidget(btn)
        # 急停是锁存的（固件 app/chassis.c 的 ESTOP），不解就是复位前一直停着 ——
        # 所以解除必须是主界面上一个够显眼的按钮，不能藏进菜单。
        self.btn_release.setStyleSheet("font-size:20px; background-color:#1e6b3b; color:white;")
        self.btn_stop.setStyleSheet("font-size:20px; font-weight:bold; background-color:#c0392b; color:white;")
        self.btn_selftest.clicked.connect(self.start_self_test); self.btn_mow.clicked.connect(self.start_mow); self.btn_follow.clicked.connect(self.toggle_follow); self.btn_dock.clicked.connect(self.start_dock); self.btn_stop.clicked.connect(self.stop_all); self.btn_release.clicked.connect(self.release_estop)

        self.btn_record = QPushButton("SLAM地图"); self.btn_record.setMinimumHeight(40); self.btn_record.setStyleSheet("font-size:20px;"); self.btn_record.clicked.connect(self.show_slam_page); left_layout.addWidget(self.btn_record)
        self.btn_settings = QPushButton("设置"); self.btn_settings.setMinimumHeight(40); self.btn_settings.setStyleSheet("font-size:20px;"); self.btn_settings.clicked.connect(self.show_settings); left_layout.addWidget(self.btn_settings)
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
        self.btn_select_all = QPushButton("全选"); self.btn_select_all.setMinimumHeight(40); self.btn_select_all.clicked.connect(self._toggle_select_all); top.addWidget(self.btn_select_all)
        self.btn_sort_video = QPushButton("时间▼"); self.btn_sort_video.setMinimumHeight(40)
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
        rbb = QPushButton("← 返回"); rbb.setFixedHeight(40); rbb.clicked.connect(self.show_speed_menu); rl.addWidget(rbb); self.stack_layout.addWidget(self.gauge_rec_widget)

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
            btn = QPushButton(text); btn.setMinimumHeight(40); btn.setStyleSheet("font-size:20px;"); btn.clicked.connect(handler); l.addWidget(btn)
            if text != buttons[-1][0]: l.addSpacing(6)
        return w

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
