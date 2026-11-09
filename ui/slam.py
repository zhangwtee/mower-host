import os
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMessageBox
from ui.const import TEST_MODE, ROS_WS, MAP_DIR, RVIZ_CFG


class SlamMixin:

    def _start_bridge(self):
        """v3：底盘桥启动即拉、常驻在线；叠加层启停不重启本层。Mock 模式跳过。"""
        import subprocess, os
        self._bridge_proc = subprocess.Popen(
            ["bash", "-c", self.BRIDGE_SETUP + " && " + self.BRIDGE_LAUNCH],
            env=dict(os.environ, DISPLAY=":0"))

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
