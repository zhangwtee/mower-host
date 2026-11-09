import os
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QHBoxLayout, QPushButton, QLabel, QMessageBox, QListWidget, QListWidgetItem, QAbstractItemView
from ui.const import TEST_MODE, MAP_DIR, RVIZ_CFG


class MaplibMixin:

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
