"""主窗口：只保留装配、任务状态与退出清理；页面/相机/建图/地图库已拆到 mixin。"""
import os
import json
from collections import deque
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QTextCursor
from ui.const import TEST_MODE, CONFIG_FILE, MAP_DIR

from ui.base import WindowBase
from ui.camera import CameraMixin
from ui.maplib import MaplibMixin
from ui.mock_slam import MockSlamMixin
from ui.pages import PagesMixin
from ui.slam import SlamMixin


class MainWindow(WindowBase, PagesMixin, CameraMixin, SlamMixin, MockSlamMixin, MaplibMixin):

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
        # 指示灯实时刷新（2 秒）：从话题状态读，不再靠一次性就绪信号
        self._led_timer = QTimer(); self._led_timer.timeout.connect(self._refresh_leds); self._led_timer.start(2000)

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

    def clear_send_log(self): self.send_log.clear()

    def clear_recv_log(self): self.recv_log.clear()

    def log_send(self, label=""): self.send_log.append(f"[{label}]"); self.send_log.moveCursor(QTextCursor.End)

    def log_recv(self, text): self.recv_log.append(text); self.recv_log.moveCursor(QTextCursor.End)

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

    def _set_led(self, label, on, tip=""):
        """单个指示灯：在 = 绿，不在 = 灰。一个灯只有这两种状态。"""
        if label is None:
            return
        color = "#00cc44" if on else "#888"
        style = f"color: {color}; font-size: 14px;"
        if label.styleSheet() != style:
            label.setStyleSheet(style)
        if tip:
            label.setToolTip(tip)

    def _refresh_leds(self):
        """按话题的当前发布者刷新五个灯（实时状态，非历史快照）。"""
        fn = getattr(self.serial_mgr, "ready_state", None)
        if not callable(fn):
            return
        try:
            st = fn() or {}
        except Exception:
            return
        self._set_led(self.led_chassis, st.get("chassis"), "/odom")
        self._set_led(self.led_lidar, st.get("lidar"), "/scan")
        self._set_led(self.led_ekf, st.get("ekf"), "/odom_combined")
        self._set_led(self.led_slam, st.get("slam"), "/map")
        self._set_led(self.led_nav, st.get("nav"), "/plan")

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
