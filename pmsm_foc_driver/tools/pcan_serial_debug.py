import can
import serial
import time
import threading

# --- 配置参数 ---
CAN_CHANNEL = 'PCAN_USBBUS1'
CAN_BITRATE = 500000
SERIAL_PORT = 'COM4'
SERIAL_BAUD = 115200
NODE_ID     = 1

# SDO 命令头
SDO_WRITE_4BYTE = 0x23
SDO_WRITE_2BYTE = 0x2B
SDO_WRITE_1BYTE = 0x2F
SDO_READ        = 0x40
SDO_RESPONSE    = 0x60

# COB-ID
TX_SDO = 0x600 + NODE_ID
RX_SDO = 0x580 + NODE_ID

# 标志位：控制线程停止
stop_event = threading.Event()

def serial_monitor():
    """串口监听线程：实时打印 MCU 的调试信息"""
    print(f"[Monitor] Starting COM4 monitor...")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        while not stop_event.is_set():
            line = ser.readline()
            if line:
                try:
                    print(f"[MCU] {line.decode('utf-8', errors='ignore').strip()}")
                except:
                    pass
        ser.close()
    except Exception as e:
        print(f"[Monitor] Error: {e}")

def combined_test():
    print(f"--- Combined CAN + Serial Debugging ---")
    
    # 启动串口监听线程
    monitor_thread = threading.Thread(target=serial_monitor, daemon=True)
    monitor_thread.start()

    try:
        bus = can.interface.Bus(interface='pcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        print(f"[CAN] Connected to PCAN")
    except Exception as e:
        print(f"[CAN] Error: {e}")
        stop_event.set()
        return

    def sdo_write(index, sub, data, length):
        cmd = SDO_WRITE_4BYTE if length == 4 else (SDO_WRITE_2BYTE if length == 2 else SDO_WRITE_1BYTE)
        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, sub]
        payload += list(data.to_bytes(4, 'little', signed=(data < 0)))
        bus.send(can.Message(arbitration_id=TX_SDO, data=payload, is_extended_id=False))
        start = time.time()
        while time.time() - start < 0.5:
            rx = bus.recv(0.01)
            if rx and rx.arbitration_id == RX_SDO and rx.data[0] == 0x60: return True
        return False

    def sdo_read(index, sub):
        bus.send(can.Message(arbitration_id=TX_SDO, data=[SDO_READ, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0], is_extended_id=False))
        start = time.time()
        while time.time() - start < 0.5:
            rx = bus.recv(0.01)
            if rx and rx.arbitration_id == RX_SDO:
                return int.from_bytes(rx.data[4:8], 'little', signed=True)
        return None

    # --- 联调逻辑开始 ---
    print("\n[Action] Configuring Motor...")
    sdo_write(0x6060, 0, 1, 1)      # PP Mode
    sdo_write(0x6081, 0, 40000, 4)  # 5 RPS (中速)

    print("[Action] Enabling Operation...")
    for val in [0x06, 0x07, 0x0F]:
        sdo_write(0x6040, 0, val, 2)
        time.sleep(0.2)

    # 触发一个大的运动
    print("[Action] Triggering 5-rev relative move...")
    target = 40000 # 5圈
    sdo_write(0x607A, 0, target, 4)
    sdo_write(0x6040, 0, 0x3F, 2) # Trigger Rel Move

    # 监控 10 秒
    print("[Info] Monitoring MCU Output for 10s...")
    start = time.time()
    while time.time() - start < 10:
        pos = sdo_read(0x6064, 0)
        sw = sdo_read(0x6041, 0)
        # 这里的打印会和串口线程的打印交织，方便定位问题
        print(f"      [Status] SW: {hex(sw if sw else 0)} | Pos: {pos}")
        time.sleep(1.0)

    # 停止并清理
    print("\n[Action] Shutting down...")
    sdo_write(0x6040, 0, 0x06, 2)
    stop_event.set()
    monitor_thread.join(timeout=1.0)
    bus.shutdown()

if __name__ == "__main__":
    combined_test()
