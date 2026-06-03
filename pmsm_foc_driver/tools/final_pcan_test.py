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

def serial_monitor(stop_event):
    """串口监听：实时打印 MCU 的调试信息"""
    print(f"[Monitor] Starting {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        while not stop_event.is_set():
            line = ser.readline()
            if line:
                try: 
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text: print(f"[MCU] {text}")
                except: pass
        ser.close()
    except Exception as e: print(f"[Serial Error] {e}")

def final_test():
    stop_event = threading.Event()
    monitor_thread = threading.Thread(target=serial_monitor, args=(stop_event,), daemon=True)
    monitor_thread.start()

    try:
        bus = can.interface.Bus(interface='pcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        print("[CAN] PCAN Connected.")
    except Exception as e:
        print(f"CAN Connection Failed: {e}")
        stop_event.set()
        return

    def sdo_write(index, sub, data, length):
        cmd = 0x23 if length == 4 else (0x2B if length == 2 else 0x2F)
        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, sub]
        payload += list(data.to_bytes(4, 'little', signed=(data < 0)))
        bus.send(can.Message(arbitration_id=0x600+NODE_ID, data=payload, is_extended_id=False))
        time.sleep(0.1)

    def sdo_read(index, sub):
        bus.send(can.Message(arbitration_id=0x600+NODE_ID, data=[0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0], is_extended_id=False))
        start = time.time()
        while time.time() - start < 0.2:
            rx = bus.recv(0.01)
            if rx and rx.arbitration_id == 0x580+NODE_ID:
                return int.from_bytes(rx.data[4:8], 'little', signed=True)
        return None

    print("\n--- Starting Final Interaction Test ---")
    
    # 1. 预配置：位置模式 + 轮廓速度 (10 RPS)
    print("[Action] Configuring Mode and Velocity...")
    sdo_write(0x6060, 0, 1, 1)      # PP Mode
    sdo_write(0x6081, 0, 80000, 4)  # 10 RPS
    
    # 2. 状态机启动
    print("[Action] Transitioning to OPERATION_ENABLED...")
    for val in [0x06, 0x07, 0x0F]:
        sdo_write(0x6040, 0, val, 2)
        time.sleep(0.5) 

    # 3. 执行序列
    print("[Action] Executing move sequence...")
    moves = [
        ("归零", 0, False),
        ("正向一圈", 8000, True),
        ("反向半圈", -4000, True),
        ("正向一圈", 8000, True),
        ("最终归零", 0, False)
    ]

    for name, target, rel in moves:
        print(f"\nMoving: {name} ({target})")
        sdo_write(0x607A, 0, int(target), 4)
        
        # CiA 402 Controlword (0x6040) bit mapping:
        # bit 4 (0x10): New Setpoint
        # bit 5 (0x20): Change Set Immediately
        # bit 6 (0x40): 0 = Absolute, 1 = Relative
        # Base state: 0x0F (Operation Enabled)
        
        if rel:
            ctrl = 0x0F | 0x10 | 0x20 | 0x40  # 0x7F: Rel + Immed + New Setpoint
        else:
            ctrl = 0x0F | 0x10 | 0x20         # 0x3F: Abs + Immed + New Setpoint
            
        sdo_write(0x6040, 0, ctrl, 2)
        
        # 监听并打印实际位置
        start = time.time()
        success = False
        while time.time() - start < 8.0:
            sw = sdo_read(0x6041, 0)
            pos = sdo_read(0x6064, 0)
            print(f"      [Pos] {pos} | SW: {hex(sw if sw else 0)}", end='\r')
            
            if sw and (sw & 0x0400): # Target Reached
                print(f"\n      Success: {name} Reached.")
                sdo_write(0x6040, 0, 0x0F, 2) # Clear setpoint bit
                success = True
                break
            time.sleep(0.1)
        
        if not success: print(f"\n      Timeout waiting for {name}")

    print("\n[Action] Shutting down...")
    sdo_write(0x6040, 0, 0x06, 2)
    time.sleep(0.5)
    stop_event.set()
    monitor_thread.join(timeout=1.0)
    bus.shutdown()
    print("--- Test Finished ---")

if __name__ == "__main__":
    final_test()
