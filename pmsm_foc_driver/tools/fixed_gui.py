#!/usr/bin/env python3
"""
CAN Master GUI - CiA 402 FIXED VERSION V4
增加功能：
1. 更新速度上限至 5000 RPM
"""

import sys
import can
import threading
import time
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QGroupBox, QGridLayout, QSpinBox, QCheckBox, QComboBox
)
from PyQt6.QtCore import pyqtSignal, QObject, QTimer


class CanMasterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bus = None
        self.node_id = 1
        self.init_ui()
        
        # 定时器用于更新实时反馈
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.poll_feedback)

    def init_ui(self):
        self.setWindowTitle("CiA 402 FIXED GUI V4 - 位置/速度全能版")
        self.setMinimumSize(800, 800)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 1. 通讯连接
        conn_group = QGroupBox("1. 通讯连接")
        conn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("连接 PCAN (500K)")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.btn_connect)
        conn_group.setLayout(conn_layout)
        main_layout.addWidget(conn_group)

        # 1.5. NMT 控制 (CiA 301)
        nmt_group = QGroupBox("1.5. NMT 控制 (CiA 301)")
        nmt_layout = QHBoxLayout()
        btn_nmt_start = QPushButton("Start (0x01)"); btn_nmt_start.clicked.connect(lambda: self.send_nmt(0x01))
        btn_nmt_preop = QPushButton("Pre-Op (0x80)"); btn_nmt_preop.clicked.connect(lambda: self.send_nmt(0x80))
        btn_nmt_reset = QPushButton("Reset Node (0x81)"); btn_nmt_reset.clicked.connect(lambda: self.send_nmt(0x81))
        btn_nmt_comm = QPushButton("Reset Comm (0x82)"); btn_nmt_comm.clicked.connect(lambda: self.send_nmt(0x82))
        nmt_layout.addWidget(btn_nmt_start)
        nmt_layout.addWidget(btn_nmt_preop)
        nmt_layout.addWidget(btn_nmt_reset)
        nmt_layout.addWidget(btn_nmt_comm)
        nmt_group.setLayout(nmt_layout)
        main_layout.addWidget(nmt_group)

        # 2. 状态控制 (CiA 402)
        state_group = QGroupBox("2. 系统控制 (CiA 402)")
        state_layout = QGridLayout()
        
        state_layout.addWidget(QLabel("控制模式:"), 0, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Profile Position (PP)", "Profile Velocity (PV)"])
        state_layout.addWidget(self.combo_mode, 0, 1)
        
        self.btn_apply_mode = QPushButton("设置模式 (0x6060)")
        self.btn_apply_mode.clicked.connect(self.apply_mode)
        state_layout.addWidget(self.btn_apply_mode, 0, 2)
        
        self.btn_shutdown = QPushButton("Shutdown (0x06)")
        self.btn_shutdown.clicked.connect(lambda: self.send_controlword(0x06))
        state_layout.addWidget(self.btn_shutdown, 1, 0)

        self.btn_switch_on = QPushButton("Switch On (0x07)")
        self.btn_switch_on.clicked.connect(lambda: self.send_controlword(0x07))
        state_layout.addWidget(self.btn_switch_on, 1, 1)

        self.btn_enable = QPushButton("Enable (0x0F)")
        self.btn_enable.clicked.connect(lambda: self.send_controlword(0x0F))
        self.btn_enable.setStyleSheet("background-color: #90EE90;")
        state_layout.addWidget(self.btn_enable, 1, 2)

        self.btn_fault_reset = QPushButton("Fault Reset (0x80)")
        self.btn_fault_reset.clicked.connect(lambda: self.send_controlword(0x80))
        state_layout.addWidget(self.btn_fault_reset, 1, 3)
        
        state_group.setLayout(state_layout)
        main_layout.addWidget(state_group)

        # 3. 参数配置
        param_group = QGroupBox("3. 参数配置")
        param_layout = QGridLayout()
        param_layout.addWidget(QLabel("轮廓速度 (0x6081, RPM):"), 0, 0)
        self.profile_vel = QSpinBox()
        self.profile_vel.setRange(1, 5000); self.profile_vel.setValue(60)
        param_layout.addWidget(self.profile_vel, 0, 1)
        self.btn_set_pvel = QPushButton("设置位置速度")
        self.btn_set_pvel.clicked.connect(self.set_profile_velocity)
        param_layout.addWidget(self.btn_set_pvel, 0, 2)
        
        param_layout.addWidget(QLabel("加速度 (0x6083, ms):"), 1, 0)
        self.profile_acc = QSpinBox()
        self.profile_acc.setRange(10, 10000); self.profile_acc.setValue(1000)
        param_layout.addWidget(self.profile_acc, 1, 1)
        self.btn_set_pacc = QPushButton("设置斜坡时间")
        self.btn_set_pacc.clicked.connect(self.set_profile_acceleration)
        param_layout.addWidget(self.btn_set_pacc, 1, 2)
        
        param_group.setLayout(param_layout)
        main_layout.addWidget(param_group)

        # 4. 运动控制
        move_group = QGroupBox("4. 运动指令")
        move_layout = QGridLayout()
        
        # PP 模式控制
        move_layout.addWidget(QLabel("PP 目标位置 (Counts):"), 0, 0)
        self.target_pos = QSpinBox()
        self.target_pos.setRange(-1000000, 1000000); self.target_pos.setValue(4000)
        move_layout.addWidget(self.target_pos, 0, 1)
        self.abs_rel = QCheckBox("绝对位置")
        self.abs_rel.setChecked(True)
        move_layout.addWidget(self.abs_rel, 0, 2)
        self.btn_move = QPushButton("触发位置运动")
        self.btn_move.clicked.connect(self.move_to)
        self.btn_move.setStyleSheet("background-color: #87CEEB; font-weight: bold; height: 35px;")
        move_layout.addWidget(self.btn_move, 1, 0, 1, 3)

        # PV 模式控制
        move_layout.addWidget(QLabel("PV 目标速度 (RPM):"), 2, 0)
        self.target_vel = QSpinBox()
        self.target_vel.setRange(-5000, 5000); self.target_vel.setValue(100)
        move_layout.addWidget(self.target_vel, 2, 1)
        self.btn_vel_run = QPushButton("运行速度模式")
        self.btn_vel_run.clicked.connect(self.velocity_run)
        self.btn_vel_run.setStyleSheet("background-color: #FFB6C1; font-weight: bold; height: 35px;")
        move_layout.addWidget(self.btn_vel_run, 2, 2, 1, 1)
        
        move_group.setLayout(move_layout)
        main_layout.addWidget(move_group)

        # 5. 实时反馈
        fb_group = QGroupBox("5. 实时反馈")
        fb_layout = QGridLayout()
        
        fb_layout.addWidget(QLabel("当前位置:"), 0, 0)
        self.lbl_pos = QLabel("---")
        self.lbl_pos.setStyleSheet("font-size: 24px; font-weight: bold; color: blue;")
        fb_layout.addWidget(self.lbl_pos, 0, 1)
        
        fb_layout.addWidget(QLabel("当前速度:"), 0, 2)
        self.lbl_vel = QLabel("---")
        self.lbl_vel.setStyleSheet("font-size: 24px; font-weight: bold; color: green;")
        fb_layout.addWidget(self.lbl_vel, 0, 3)
        
        fb_group.setLayout(fb_layout)
        main_layout.addWidget(fb_group)

        # 日志
        self.log_win = QTextEdit(); self.log_win.setReadOnly(True)
        main_layout.addWidget(self.log_win)

    def log(self, text): self.log_win.append(f"[{time.strftime('%H:%M:%S')}] {text}")

    def toggle_connection(self):
        if self.bus is None:
            try:
                self.bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
                self.log("PCAN 已连接 (500K)")
                self.send_frame(0x000, [0x01, self.node_id]) # NMT Start
                self.btn_connect.setText("断开 PCAN")
                self.update_timer.start(100) # 100ms 刷新一次
            except Exception as e: self.log(f"连接失败: {e}")
        else:
            self.update_timer.stop()
            self.bus.shutdown(); self.bus = None
            self.btn_connect.setText("连接 PCAN (500K)")
            self.log("PCAN 已断开")

    def send_frame(self, can_id, data):
        if self.bus:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg)

    def send_sdo_write(self, index, subindex, val, length):
        cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[length]
        data = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]
        v_bytes = val.to_bytes(4, 'little', signed=True)
        data += list(v_bytes[:4])
        self.send_frame(0x600 + self.node_id, data)

    def send_sdo_read(self, index, subindex):
        self.send_frame(0x600 + self.node_id, [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])

    def send_nmt(self, cs):
        self.send_frame(0x000, [cs, self.node_id])
        self.log(f"发送 NMT 指令: 0x{cs:02X}")

    def send_controlword(self, cw):
        self.send_sdo_write(0x6040, 0, cw, 2)
        self.log(f"下发控制字 0x6040: 0x{cw:04X}")

    def apply_mode(self):
        mode_str = self.combo_mode.currentText()
        mode_val = 1 if "PP" in mode_str else 3
        self.send_sdo_write(0x6060, 0, mode_val, 1)
        self.log(f"设置目标模式 0x6060: {mode_str} ({mode_val})")
        # 可以选择同步下发速度/加速度等参数
        self.set_profile_velocity()
        self.set_profile_acceleration()

    def set_profile_velocity(self):
        vel = self.profile_vel.value()
        self.send_sdo_write(0x6081, 0, vel, 4)
        self.log(f"设置 PP 轮廓速度: {vel} RPM")
        
    def set_profile_acceleration(self):
        acc = self.profile_acc.value()
        # 我们用 0x6083 暂代斜坡时间 (单位 ms)
        self.send_sdo_write(0x6083, 0, acc, 4)
        self.log(f"设置斜坡时间: {acc} ms")

    def move_to(self):
        pos = self.target_pos.value()
        self.send_sdo_write(0x607A, 0, pos, 4)
        
        is_rel = not self.abs_rel.isChecked()
        cw_base = 0x002F # Bit 5 = 1 (Change Set Immed)
        if is_rel: cw_base |= 0x0040 # Bit 6 = 1 (Rel)
        
        # 触发上升沿
        self.send_sdo_write(0x6040, 0, cw_base & ~0x10, 2); time.sleep(0.02)
        self.send_sdo_write(0x6040, 0, cw_base | 0x10, 2); time.sleep(0.02)
        self.send_sdo_write(0x6040, 0, cw_base & ~0x10, 2)
        self.log(f"下发位置指令: {pos} counts (%s)" % ("相对" if is_rel else "绝对"))

    def velocity_run(self):
        vel = self.target_vel.value()
        self.send_sdo_write(0x60FF, 0, vel, 4)
        self.log(f"下发速度指令: {vel} RPM")

    def poll_feedback(self):
        if not self.bus: return
        # 依次请求位置和速度
        self.send_sdo_read(0x6064, 0)
        time.sleep(0.02)
        self.send_sdo_read(0x606C, 0)
        
        # 尝试接收 SDO 响应
        for _ in range(5):
            try:
                msg = self.bus.recv(timeout=0.01)
                if msg and msg.arbitration_id == 0x580 + self.node_id:
                    index = msg.data[1] + (msg.data[2] << 8)
                    val = int.from_bytes(msg.data[4:8], 'little', signed=True)
                    if index == 0x6064:
                        self.lbl_pos.setText(str(val))
                    elif index == 0x606C:
                        self.lbl_vel.setText(str(val))
            except: break

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CanMasterWindow(); window.show()
    sys.exit(app.exec())
