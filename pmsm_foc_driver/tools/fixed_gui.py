#!/usr/bin/env python3
"""
CAN Master GUI - CiA 402 FIXED VERSION V5
改动:
- 移除实时轮询（速度/位置）
- 新增手动 CAN 指令收发窗口
- 后台线程持续接收 CAN 报文并显示
"""

import sys
import can
import threading
import time
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QGroupBox, QGridLayout, QSpinBox, QCheckBox, QComboBox, QLineEdit
)
from PyQt6.QtCore import pyqtSignal, QObject, QTimer, Qt


# ── 后台 CAN 接收线程 ──────────────────────────────────────────
class CanReceiver(QObject):
    """持续监听 CAN 总线，通过信号把收到的帧发给主线程"""
    msg_received = pyqtSignal(int, list)  # can_id, data (list of ints)

    def __init__(self, bus_getter):
        super().__init__()
        self._bus_getter = bus_getter  # callable, returns bus or None
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)

    def _run(self):
        while self._running:
            bus = self._bus_getter()
            if bus is None:
                time.sleep(0.1)
                continue
            try:
                msg = bus.recv(timeout=0.1)
                if msg is not None:
                    # 只转发 11-bit ID 的标准帧
                    data_list = list(msg.data) if msg.data else []
                    self.msg_received.emit(msg.arbitration_id, data_list)
            except Exception:
                pass


# ── 主窗口 ─────────────────────────────────────────────────────
class CanMasterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bus = None
        self.node_id = 1           # 默认 Node ID (与固件一致 NodeID=1)
        self.can_receiver = CanReceiver(lambda: self.bus)
        self.can_receiver.msg_received.connect(self.on_can_received)
        self.init_ui()

    # ── UI 构建 ─────────────────────────────────────────────────
    def init_ui(self):
        self.setWindowTitle("CiA 402 CAN Tool V5 — 手动指令收发")
        self.setMinimumSize(900, 850)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ──── 1. 通讯连接 ────
        conn_group = QGroupBox("1. 通讯连接")
        conn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("连接 PCAN (500K)")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.btn_connect)

        conn_layout.addWidget(QLabel("  Node ID (hex):"))
        self.node_id_spin = QSpinBox()
        self.node_id_spin.setRange(1, 127)
        self.node_id_spin.setValue(self.node_id)
        self.node_id_spin.setPrefix("0x")
        self.node_id_spin.setDisplayIntegerBase(16)
        self.node_id_spin.valueChanged.connect(lambda v: setattr(self, 'node_id', v))
        conn_layout.addWidget(self.node_id_spin)
        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)
        main_layout.addWidget(conn_group)

        # ──── 1.5. NMT 控制 ────
        nmt_group = QGroupBox("1.5. NMT 控制 (CiA 301)")
        nmt_layout = QHBoxLayout()
        btn_nmt_start  = QPushButton("Start (0x01)");  btn_nmt_start.clicked.connect(lambda: self.send_nmt(0x01))
        btn_nmt_preop  = QPushButton("Pre-Op (0x80)"); btn_nmt_preop.clicked.connect(lambda: self.send_nmt(0x80))
        btn_nmt_reset  = QPushButton("Reset Node (0x81)"); btn_nmt_reset.clicked.connect(lambda: self.send_nmt(0x81))
        btn_nmt_comm   = QPushButton("Reset Comm (0x82)"); btn_nmt_comm.clicked.connect(lambda: self.send_nmt(0x82))
        nmt_layout.addWidget(btn_nmt_start)
        nmt_layout.addWidget(btn_nmt_preop)
        nmt_layout.addWidget(btn_nmt_reset)
        nmt_layout.addWidget(btn_nmt_comm)
        nmt_group.setLayout(nmt_layout)
        main_layout.addWidget(nmt_group)

        # ──── 2. 状态控制 (CiA 402) ────
        state_group = QGroupBox("2. 系统控制 (CiA 402)")
        state_layout = QGridLayout()

        state_layout.addWidget(QLabel("控制模式:"), 0, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Profile Position (PP)", "Profile Velocity (PV)", "Homing (HM)"])
        state_layout.addWidget(self.combo_mode, 0, 1)

        self.btn_apply_mode = QPushButton("设置模式 (0x6060)")
        self.btn_apply_mode.clicked.connect(self.apply_mode)
        state_layout.addWidget(self.btn_apply_mode, 0, 2)

        self.btn_shutdown   = QPushButton("Shutdown (0x06)");   self.btn_shutdown.clicked.connect(lambda: self.send_controlword(0x06))
        self.btn_switch_on  = QPushButton("Switch On (0x07)");  self.btn_switch_on.clicked.connect(lambda: self.send_controlword(0x07))
        self.btn_enable     = QPushButton("Enable (0x0F)");     self.btn_enable.clicked.connect(lambda: self.send_controlword(0x0F))
        self.btn_enable.setStyleSheet("background-color: #90EE90;")

        self.btn_quick_stop = QPushButton("Quick Stop (0x02)")
        self.btn_quick_stop.clicked.connect(lambda: self.send_controlword(0x02))
        self.btn_fault_reset = QPushButton("Fault Reset (0x80)")
        self.btn_fault_reset.clicked.connect(lambda: self.send_controlword(0x80))

        state_layout.addWidget(self.btn_shutdown,    1, 0)
        state_layout.addWidget(self.btn_switch_on,   1, 1)
        state_layout.addWidget(self.btn_enable,      1, 2)
        state_layout.addWidget(self.btn_quick_stop,  2, 0)
        state_layout.addWidget(self.btn_fault_reset, 2, 1)

        # 回零专用按钮
        self.btn_hm_start = QPushButton("启动回零 (bit4=1)")
        self.btn_hm_start.clicked.connect(lambda: self.send_controlword(0x1F))
        self.btn_hm_start.setStyleSheet("background-color: #FFD700; font-weight: bold;")
        state_layout.addWidget(self.btn_hm_start, 2, 2)

        state_group.setLayout(state_layout)
        main_layout.addWidget(state_group)

        # ──── 3. 参数配置 ────
        param_group = QGroupBox("3. 参数配置 (SDO)")
        param_layout = QGridLayout()

        param_layout.addWidget(QLabel("轮廓速度 (0x6081, RPM):"), 0, 0)
        self.profile_vel = QSpinBox()
        self.profile_vel.setRange(1, 5000); self.profile_vel.setValue(60)
        param_layout.addWidget(self.profile_vel, 0, 1)
        self.btn_set_pvel = QPushButton("设置")
        self.btn_set_pvel.clicked.connect(self.set_profile_velocity)
        param_layout.addWidget(self.btn_set_pvel, 0, 2)

        param_layout.addWidget(QLabel("加速度 (0x6083, RPM/s):"), 1, 0)
        self.profile_acc = QSpinBox()
        self.profile_acc.setRange(10, 100000); self.profile_acc.setValue(3000)
        param_layout.addWidget(self.profile_acc, 1, 1)
        self.btn_set_pacc = QPushButton("设置")
        self.btn_set_pacc.clicked.connect(self.set_profile_acceleration)
        param_layout.addWidget(self.btn_set_pacc, 1, 2)

        # 回零参数
        param_layout.addWidget(QLabel("回零速度 sub1/2 (0x6099):"), 2, 0)
        self.hm_speed1 = QSpinBox(); self.hm_speed1.setRange(0, 65535); self.hm_speed1.setValue(0)
        param_layout.addWidget(self.hm_speed1, 2, 1)
        self.hm_speed2 = QSpinBox(); self.hm_speed2.setRange(0, 65535); self.hm_speed2.setValue(0)
        param_layout.addWidget(self.hm_speed2, 2, 2)
        self.btn_hm_speed = QPushButton("设置回零速度")
        self.btn_hm_speed.clicked.connect(self.set_homing_speed)
        param_layout.addWidget(self.btn_hm_speed, 2, 3)

        param_layout.addWidget(QLabel("回零加速度 (0x609A):"), 3, 0)
        self.hm_acc = QSpinBox(); self.hm_acc.setRange(0, 65535); self.hm_acc.setValue(0)
        param_layout.addWidget(self.hm_acc, 3, 1)
        self.btn_hm_acc = QPushButton("设置")
        self.btn_hm_acc.clicked.connect(self.set_homing_accel)
        param_layout.addWidget(self.btn_hm_acc, 3, 2)

        param_layout.addWidget(QLabel("回零方式 (0x6098):"), 4, 0)
        self.hm_method = QSpinBox(); self.hm_method.setRange(0, 255); self.hm_method.setValue(35)
        param_layout.addWidget(self.hm_method, 4, 1)
        self.btn_hm_method = QPushButton("设置")
        self.btn_hm_method.clicked.connect(self.set_homing_method)
        param_layout.addWidget(self.btn_hm_method, 4, 2)

        param_layout.addWidget(QLabel("原点偏置 (0x607C):"), 5, 0)
        self.home_offset = QSpinBox(); self.home_offset.setRange(-1000000, 1000000); self.home_offset.setValue(0)
        param_layout.addWidget(self.home_offset, 5, 1)
        self.btn_hm_offset = QPushButton("设置")
        self.btn_hm_offset.clicked.connect(self.set_home_offset)
        param_layout.addWidget(self.btn_hm_offset, 5, 2)

        param_group.setLayout(param_layout)
        main_layout.addWidget(param_group)

        # ──── 4. 运动指令 ────
        move_group = QGroupBox("4. 运动指令")
        move_layout = QGridLayout()

        # PP
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

        # PV
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

        # ──── 4.5. 回零操作 ────
        hm_op_group = QGroupBox("4.5. 回零操作 (Homing)")
        hm_op_layout = QGridLayout()

        hm_op_layout.addWidget(QLabel("回零速度 (RPM):"), 0, 0)
        self.hm_op_speed = QSpinBox(); self.hm_op_speed.setRange(1, 500); self.hm_op_speed.setValue(60)
        hm_op_layout.addWidget(self.hm_op_speed, 0, 1)

        hm_op_layout.addWidget(QLabel("方法 (33-37):"), 0, 2)
        self.hm_op_method = QSpinBox(); self.hm_op_method.setRange(33, 37); self.hm_op_method.setValue(35)
        hm_op_layout.addWidget(self.hm_op_method, 0, 3)

        self.btn_hm_full = QPushButton("▶ 一键回零 (Enable + HM + Start)")
        self.btn_hm_full.clicked.connect(self.homing_sequence)
        self.btn_hm_full.setStyleSheet("background-color: #FFD700; font-weight: bold; height: 30px;")
        hm_op_layout.addWidget(self.btn_hm_full, 1, 0, 1, 4)

        self.btn_hm_start_only = QPushButton("仅启动回零 (bit4=1)")
        self.btn_hm_start_only.clicked.connect(self.homing_start_only)
        self.btn_hm_start_only.setStyleSheet("background-color: #FFA500; font-weight: bold;")
        hm_op_layout.addWidget(self.btn_hm_start_only, 2, 0, 1, 2)

        self.btn_hm_read_pos = QPushButton("读取位置 (0x6064)")
        self.btn_hm_read_pos.clicked.connect(lambda: self.send_sdo_read(0x6064, 0))
        hm_op_layout.addWidget(self.btn_hm_read_pos, 2, 2, 1, 2)

        self.lbl_hm_status = QLabel("状态: ---")
        self.lbl_hm_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #333;")
        hm_op_layout.addWidget(self.lbl_hm_status, 3, 0, 1, 4)

        hm_op_group.setLayout(hm_op_layout)
        main_layout.addWidget(hm_op_group)

        # ──── 5. SDO 快速读写 ────
        sdo_group = QGroupBox("5. SDO 快速读写")
        sdo_layout = QGridLayout()

        sdo_layout.addWidget(QLabel("Index (hex):"), 0, 0)
        self.sdo_index = QLineEdit("6041")
        self.sdo_index.setPlaceholderText("e.g. 6041")
        sdo_layout.addWidget(self.sdo_index, 0, 1)

        sdo_layout.addWidget(QLabel("Sub:"), 0, 2)
        self.sdo_sub = QSpinBox(); self.sdo_sub.setRange(0, 255); self.sdo_sub.setValue(0)
        sdo_layout.addWidget(self.sdo_sub, 0, 3)

        sdo_layout.addWidget(QLabel("Value (hex):"), 0, 4)
        self.sdo_value = QLineEdit("0")
        self.sdo_value.setPlaceholderText("e.g. 0F")
        sdo_layout.addWidget(self.sdo_value, 0, 5)

        self.btn_sdo_read  = QPushButton("SDO 读取");  self.btn_sdo_read.clicked.connect(self.sdo_read)
        self.btn_sdo_read.setStyleSheet("background-color: #98FB98;")
        self.btn_sdo_write = QPushButton("SDO 写入"); self.btn_sdo_write.clicked.connect(self.sdo_write)
        self.btn_sdo_write.setStyleSheet("background-color: #FFB6C1;")
        sdo_layout.addWidget(self.btn_sdo_read,  1, 0, 1, 3)
        sdo_layout.addWidget(self.btn_sdo_write, 1, 3, 1, 3)

        sdo_group.setLayout(sdo_layout)
        main_layout.addWidget(sdo_group)

        # ──── 6. 手动 CAN 指令 ────
        manual_group = QGroupBox("6. 手动 CAN 指令 (Raw Frame)")
        manual_layout = QGridLayout()

        manual_layout.addWidget(QLabel("CAN ID (hex):"), 0, 0)
        self.manual_can_id = QLineEdit("601")
        self.manual_can_id.setPlaceholderText("e.g. 601")
        manual_layout.addWidget(self.manual_can_id, 0, 1)

        manual_layout.addWidget(QLabel("DLC:"), 0, 2)
        self.manual_dlc = QSpinBox(); self.manual_dlc.setRange(0, 8); self.manual_dlc.setValue(8)
        manual_layout.addWidget(self.manual_dlc, 0, 3)

        # 8 个字节
        self.manual_bytes = []
        byte_layout = QHBoxLayout()
        for i in range(8):
            le = QLineEdit("00")
            le.setMaximumWidth(35); le.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.manual_bytes.append(le)
            byte_layout.addWidget(QLabel(f"[{i}]"))
            byte_layout.addWidget(le)
        manual_layout.addLayout(byte_layout, 1, 0, 1, 4)

        btn_layout = QHBoxLayout()
        self.btn_manual_send = QPushButton("发送 CAN 帧")
        self.btn_manual_send.clicked.connect(self.manual_send)
        self.btn_manual_send.setStyleSheet("background-color: #FFD700; font-weight: bold;")
        btn_layout.addWidget(self.btn_manual_send)

        self.btn_manual_send_rtr = QPushButton("发送 Remote 帧")
        self.btn_manual_send_rtr.clicked.connect(self.manual_send_rtr)
        btn_layout.addWidget(self.btn_manual_send_rtr)

        self.chk_show_sent = QCheckBox("日志记录已发送"); self.chk_show_sent.setChecked(True)
        btn_layout.addWidget(self.chk_show_sent)
        btn_layout.addStretch()
        manual_layout.addLayout(btn_layout, 2, 0, 1, 4)

        manual_group.setLayout(manual_layout)
        main_layout.addWidget(manual_group)

        # ──── 6.5. 快捷 RAW 回零指令 ────
        raw_hm_group = QGroupBox("6.5. 快捷 RAW 回零指令")
        raw_hm_layout = QGridLayout()

        btn_raw_speed = QPushButton("回零速度")
        btn_raw_speed.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x23, 0x99, 0x60, 0x02, 0x00, 0x00, 0x01, 0x00]))
        raw_hm_layout.addWidget(btn_raw_speed, 0, 0)

        btn_raw_acc = QPushButton("回零加速度")
        btn_raw_acc.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x23, 0x9A, 0x60, 0x00, 0x20, 0x00, 0x00, 0x00]))
        raw_hm_layout.addWidget(btn_raw_acc, 0, 1)

        btn_raw_offset = QPushButton("原点偏置")
        btn_raw_offset.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x23, 0x7C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]))
        raw_hm_layout.addWidget(btn_raw_offset, 0, 2)

        btn_raw_way = QPushButton("回零方式 (33)")
        btn_raw_way.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x2F, 0x98, 0x60, 0x00, 0x21, 0x00, 0x00, 0x00]))
        raw_hm_layout.addWidget(btn_raw_way, 0, 3)

        btn_raw_mode = QPushButton("回零模式")
        btn_raw_mode.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x2F, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]))
        raw_hm_layout.addWidget(btn_raw_mode, 1, 0)

        btn_raw_start = QPushButton("开始回零 (RPDO 0x201)")
        btn_raw_start.clicked.connect(lambda: self.send_frame(0x200 + self.node_id, [0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        raw_hm_layout.addWidget(btn_raw_start, 1, 1)

        btn_raw_check = QPushButton("检查位置")
        btn_raw_check.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]))
        raw_hm_layout.addWidget(btn_raw_check, 1, 2)

        raw_hm_group.setLayout(raw_hm_layout)
        main_layout.addWidget(raw_hm_group)

        # ──── 6.7. 快捷 RAW PP 运动指令 ────
        raw_pp_group = QGroupBox("6.7. 快捷 RAW PP 运动指令")
        raw_pp_layout = QGridLayout()

        btn_pp_speed = QPushButton("1. PP 速度 (43548)")
        btn_pp_speed.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x23, 0x81, 0x60, 0x00, 0x1C, 0xAA, 0x00, 0x00]))
        raw_pp_layout.addWidget(btn_pp_speed, 0, 0)

        btn_pp_acc = QPushButton("2. PP 加速度 (32)")
        btn_pp_acc.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x23, 0x83, 0x60, 0x00, 0x20, 0x00, 0x00, 0x00]))
        raw_pp_layout.addWidget(btn_pp_acc, 0, 1)

        btn_pp_target = QPushButton("3. 目标位置 (4000)")
        btn_pp_target.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x23, 0x7A, 0x60, 0x00, 0xA0, 0x0F, 0x00, 0x00]))
        raw_pp_layout.addWidget(btn_pp_target, 0, 2)

        btn_pp_start = QPushButton("4. 启动运动 (0x1F)")
        btn_pp_start.clicked.connect(lambda: self.send_frame(0x200 + self.node_id, [0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        raw_pp_layout.addWidget(btn_pp_start, 1, 0)

        btn_pp_ret = QPushButton("5. 复位握手 (0x0F)")
        btn_pp_ret.clicked.connect(lambda: self.send_frame(0x200 + self.node_id, [0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        raw_pp_layout.addWidget(btn_pp_ret, 1, 1)

        btn_pp_check = QPushButton("6. 检查位置 (0x6064)")
        btn_pp_check.clicked.connect(lambda: self.send_frame(0x600 + self.node_id, [0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]))
        raw_pp_layout.addWidget(btn_pp_check, 1, 2)

        raw_pp_group.setLayout(raw_pp_layout)
        main_layout.addWidget(raw_pp_group)

        # ──── 7. 日志 / 接收区 ────
        log_group = QGroupBox("7. 日志 & CAN 接收")
        log_layout = QVBoxLayout()
        self.log_win = QTextEdit()
        self.log_win.setReadOnly(True)
        log_layout.addWidget(self.log_win)

        # 底部按钮
        ctrl_layout = QHBoxLayout()
        self.chk_show_rx = QCheckBox("显示接收帧"); self.chk_show_rx.setChecked(True)
        ctrl_layout.addWidget(self.chk_show_rx)
        self.chk_filter_hb = QCheckBox("屏蔽心跳包"); self.chk_filter_hb.setChecked(True)
        ctrl_layout.addWidget(self.chk_filter_hb)
        self.btn_clear_log = QPushButton("清空"); self.btn_clear_log.clicked.connect(self.log_win.clear)
        ctrl_layout.addWidget(self.btn_clear_log)
        ctrl_layout.addStretch()
        log_layout.addLayout(ctrl_layout)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

    # ── 日志 ────────────────────────────────────────────────────
    def log(self, text):
        self.log_win.append(f"[{time.strftime('%H:%M:%S')}] {text}")

    def on_can_received(self, can_id, data):
        """后台线程回调 — 显示收到的 CAN 帧"""
        if not self.chk_show_rx.isChecked():
            return
        # 心跳包过滤: CANopen Heartbeat COB-ID = 0x700 + NodeID (范围 0x700-0x77F)
        if self.chk_filter_hb.isChecked() and 0x700 <= can_id <= 0x77F:
            return
        hex_str = " ".join(f"{b:02X}" for b in data)
        self.log_win.append(
            f"<span style='color:#228B22;'>RX 0x{can_id:03X}</span>  [{hex_str}]"
        )

    # ── 连接管理 ────────────────────────────────────────────────
    def toggle_connection(self):
        if self.bus is None:
            try:
                self.bus = can.interface.Bus(
                    interface='pcan', channel='PCAN_USBBUS1', bitrate=500000
                )
                self.log("PCAN 已连接 (500K)")
                self.can_receiver.start()
                self.btn_connect.setText("断开 PCAN")
            except Exception as e:
                self.log(f"<span style='color:red;'>连接失败: {e}</span>")
        else:
            self.can_receiver.stop()
            self.bus.shutdown()
            self.bus = None
            self.btn_connect.setText("连接 PCAN (500K)")
            self.log("PCAN 已断开")

    # ── 底层发送 ────────────────────────────────────────────────
    def send_frame(self, can_id, data):
        if self.bus:
            msg = can.Message(
                arbitration_id=can_id, data=data, is_extended_id=False
            )
            self.bus.send(msg)
            if self.chk_show_sent.isChecked():
                hex_str = " ".join(f"{b:02X}" for b in data)
                self.log(f"<span style='color:#B22222;'>TX 0x{can_id:03X}</span>  [{hex_str}]")

    def send_sdo_write(self, index, subindex, val, length):
        cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[length]
        data = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]
        v_bytes = val.to_bytes(4, 'little', signed=True)
        data += list(v_bytes[:4])
        self.send_frame(0x600 + self.node_id, data)

    def send_sdo_read(self, index, subindex):
        self.send_frame(
            0x600 + self.node_id,
            [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        )

    # ── NMT / Controlword ────────────────────────────────────────
    def send_nmt(self, cs):
        self.send_frame(0x000, [cs, self.node_id])
        self.log(f"发送 NMT: 0x{cs:02X} → Node 0x{self.node_id:02X}")

    def send_controlword(self, cw):
        self.send_sdo_write(0x6040, 0, cw, 2)
        self.log(f"下发控制字 0x6040 ← 0x{cw:04X}")

    # ── 模式 ────────────────────────────────────────────────────
    def apply_mode(self):
        mode_str = self.combo_mode.currentText()
        if "PP" in mode_str:
            mode_val = 1
        elif "PV" in mode_str:
            mode_val = 3
        else:
            mode_val = 6   # HM
        self.send_sdo_write(0x6060, 0, mode_val, 1)
        self.log(f"设置模式 0x6060 ← {mode_val} ({mode_str})")

    # ── 参数 ────────────────────────────────────────────────────
    def set_profile_velocity(self):
        vel = self.profile_vel.value()
        self.send_sdo_write(0x6081, 0, vel, 4)
        self.log(f"设置轮廓速度 0x6081 ← {vel} RPM")

    def set_profile_acceleration(self):
        acc = self.profile_acc.value()
        self.send_sdo_write(0x6083, 0, acc, 4)
        self.log(f"设置加速度 0x6083 ← {acc}")

    def set_homing_speed(self):
        """写入 0x6099 sub1 和 sub2"""
        s1 = self.hm_speed1.value()
        s2 = self.hm_speed2.value()
        self.send_sdo_write(0x6099, 1, s1, 4)
        self.log(f"设置回零速度 sub1 ← {s1}")
        self.send_sdo_write(0x6099, 2, s2, 4)
        self.log(f"设置回零速度 sub2 ← {s2}")

    def set_homing_accel(self):
        acc = self.hm_acc.value()
        self.send_sdo_write(0x609A, 0, acc, 4)
        self.log(f"设置回零加速度 0x609A ← {acc}")

    def set_homing_method(self):
        m = self.hm_method.value()
        self.send_sdo_write(0x6098, 0, m, 1)
        self.log(f"设置回零方式 0x6098 ← {m}")

    def set_home_offset(self):
        off = self.home_offset.value()
        self.send_sdo_write(0x607C, 0, off, 4)
        self.log(f"设置原点偏置 0x607C ← {off}")

    # ── 回零操作 ────────────────────────────────────────────────
    def homing_start_only(self):
        """发送 CW bit4 上升沿启动回零（电机需已在 HM 模式且已使能）"""
        self.send_controlword(0x0F)         # bit4=0, 清零
        time.sleep(0.03)
        self.send_controlword(0x1F)         # bit4=1, 上升沿触发
        self.lbl_hm_status.setText("状态: 回零启动中...")
        self.log("发送回零启动触发 (bit4 0→1)")

    def homing_sequence(self):
        """完整回零流程: 设参数 → HM模式 → 使能 → 触发回零"""
        speed = self.hm_op_speed.value()
        method = self.hm_op_method.value()

        self.lbl_hm_status.setText("状态: 序列启动...")
        self.log("========== 开始回零流程 ==========")

        # 1. NMT Start
        self.send_nmt(0x01)
        time.sleep(0.05)

        # 2. 设置回零参数
        self.send_sdo_write(0x6098, 0, method, 1)       # 回零方式
        self.log(f"  回零方式: {method}")
        self.send_sdo_write(0x6099, 2, speed, 4)         # 回零速度 sub2
        self.log(f"  回零速度: {speed} RPM")
        self.send_sdo_write(0x609A, 0, 100, 4)           # 回零加速度
        self.send_sdo_write(0x607C, 0, 0, 4)             # 原点偏置=0
        time.sleep(0.05)

        # 3. 设为回零模式
        self.send_sdo_write(0x6060, 0, 6, 1)
        self.log("  模式设为 HM (6)")
        time.sleep(0.05)

        # 4. 使能电机: Shutdown → Switch On → Enable
        self.send_controlword(0x06)   # Shutdown
        time.sleep(0.05)
        self.send_controlword(0x07)   # Switch On
        time.sleep(0.05)
        self.send_controlword(0x0F)   # Enable Operation
        self.log("  电机使能中 (等待启动)...")
        time.sleep(0.5)  # 等待电机完成启动（校准+对齐）

        # 5. 启动回零: bit4 0→1 上升沿
        self.send_controlword(0x0F)   # bit4=0
        time.sleep(0.03)
        self.send_controlword(0x1F)   # bit4=1 → 触发回零
        self.lbl_hm_status.setText("状态: 回零搜索中...")
        self.log("  回零启动触发 (CW bit4 0→1)")

        # 6. 轮询等待回零完成
        self._hm_poll_count = 0
        self._hm_poll_timer = QTimer()
        self._hm_poll_timer.timeout.connect(self._poll_homing_status)
        self._hm_poll_timer.start(200)  # 每 200ms 查一次

    def _poll_homing_status(self):
        """轮询状态字 bit12(回零完成) 和 bit10(目标到达)"""
        self._hm_poll_count += 1
        if self._hm_poll_count > 100:  # 20 秒超时
            self._hm_poll_timer.stop()
            self.lbl_hm_status.setText("状态: 超时!")
            self.log("<span style='color:red;'>回零超时 (>20s)</span>")
            return

        # 读状态字 0x6041
        self.send_sdo_read(0x6041, 0)
        time.sleep(0.05)

        # 尝试收响应
        try:
            if self.bus:
                msg = self.bus.recv(timeout=0.05)
                if msg and msg.arbitration_id == 0x580 + self.node_id:
                    data = msg.data
                    if data[0] == 0x4B:  # SDO upload response, 2 bytes
                        sw = data[4] | (data[5] << 8)
                    elif data[0] == 0x43:  # SDO upload response, 4 bytes
                        sw = data[4] | (data[5] << 8)
                    else:
                        return

                    bit12 = (sw >> 12) & 1  # homing attained
                    bit10 = (sw >> 10) & 1  # target reached
                    bit13 = (sw >> 13) & 1  # homing error

                    if bit13:
                        self._hm_poll_timer.stop()
                        self.lbl_hm_status.setText("状态: 回零错误!")
                        self.log(f"<span style='color:red;'>回零错误! SW=0x{sw:04X}</span>")
                    elif bit12 and bit10:
                        self._hm_poll_timer.stop()
                        self.lbl_hm_status.setText("状态: 回零完成 ✓")
                        self.log(f"<span style='color:green;'>回零成功完成! SW=0x{sw:04X}</span>")
                        # 读回实际位置
                        self.send_sdo_read(0x6064, 0)
                    else:
                        elapsed = self._hm_poll_count * 0.2
                        self.lbl_hm_status.setText(f"状态: 搜索中... ({elapsed:.1f}s)")
        except Exception:
            pass

    # ── 运动指令 ────────────────────────────────────────────────
    def move_to(self):
        pos = self.target_pos.value()
        self.send_sdo_write(0x607A, 0, pos, 4)

        is_rel = not self.abs_rel.isChecked()
        cw_base = 0x002F  # Bit 5 = 1 (Change Set Immed)
        if is_rel:
            cw_base |= 0x0040  # Bit 6 = 1 (Rel)

        self.send_sdo_write(0x6040, 0, cw_base & ~0x10, 2)
        time.sleep(0.02)
        self.send_sdo_write(0x6040, 0, cw_base | 0x10, 2)
        time.sleep(0.02)
        self.send_sdo_write(0x6040, 0, cw_base & ~0x10, 2)
        self.log(f"下发位置指令: {pos} counts (%s)" % ("相对" if is_rel else "绝对"))

    def velocity_run(self):
        vel = self.target_vel.value()
        self.send_sdo_write(0x60FF, 0, vel, 4)
        self.log(f"下发速度指令 0x60FF ← {vel} RPM")

    # ── SDO 快速读写 ───────────────────────────────────────────
    def sdo_read(self):
        try:
            idx = int(self.sdo_index.text().strip(), 16)
        except ValueError:
            self.log("<span style='color:red;'>无效 Index</span>")
            return
        sub = self.sdo_sub.value()
        self.send_sdo_read(idx, sub)
        self.log(f"SDO 读取: 0x{idx:04X}:{sub}")

    def sdo_write(self):
        try:
            idx = int(self.sdo_index.text().strip(), 16)
        except ValueError:
            self.log("<span style='color:red;'>无效 Index</span>")
            return
        sub = self.sdo_sub.value()
        try:
            val = int(self.sdo_value.text().strip(), 16)
        except ValueError:
            self.log("<span style='color:red;'>无效 Value</span>")
            return

        # 自动决定长度（注意 0x2F 只能写 1 字节）
        abs_val = abs(val)
        if abs_val <= 0xFF:
            length = 1
        elif abs_val <= 0xFFFF:
            length = 2
        else:
            length = 4

        self.send_sdo_write(idx, sub, val, length)
        self.log(f"SDO 写入: 0x{idx:04X}:{sub} ← 0x{val:X} ({length} bytes)")

    # ── 手动 CAN 指令 ──────────────────────────────────────────
    def manual_send(self):
        try:
            can_id = int(self.manual_can_id.text().strip(), 16)
        except ValueError:
            self.log("<span style='color:red;'>无效 CAN ID</span>")
            return

        dlc = self.manual_dlc.value()
        data = []
        for i in range(dlc):
            try:
                b = int(self.manual_bytes[i].text().strip(), 16) & 0xFF
            except ValueError:
                b = 0
            data.append(b)

        # 补零到 8 字节（CAN 帧最小 data 长度在某些驱动中需要补齐）
        while len(data) < dlc:
            data.append(0)

        # CAN 消息 data 长度 = DLC
        self.send_frame(can_id, data)

    def manual_send_rtr(self):
        try:
            can_id = int(self.manual_can_id.text().strip(), 16)
        except ValueError:
            self.log("<span style='color:red;'>无效 CAN ID</span>")
            return
        dlc = self.manual_dlc.value()
        if self.bus:
            msg = can.Message(
                arbitration_id=can_id, data=[0]*dlc,
                is_extended_id=False, is_remote_frame=True
            )
            self.bus.send(msg)
            self.log(f"TX Remote 0x{can_id:03X}  DLC={dlc}")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CanMasterWindow()
    window.show()
    sys.exit(app.exec())
