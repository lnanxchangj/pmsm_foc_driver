import can
import time

# --- 配置参数 ---
CAN_CHANNEL = 'PCAN_USBBUS1'
CAN_BITRATE = 500000
NODE_ID     = 1

def torque_diagnosis():
    print("\n--- Position Mode Torque Diagnosis ---")
    try:
        bus = can.interface.Bus(interface='pcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"CAN Error: {e}")
        return

    def sdo_write(index, sub, data, length):
        cmd = 0x23 if length == 4 else (0x2B if length == 2 else 0x2F)
        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, sub]
        payload += list(data.to_bytes(4, 'little', signed=(data < 0)))
        bus.send(can.Message(arbitration_id=0x600+NODE_ID, data=payload, is_extended_id=False))
        time.sleep(0.05)

    def sdo_read(index, sub):
        bus.send(can.Message(arbitration_id=0x600+NODE_ID, data=[0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0], is_extended_id=False))
        start = time.time()
        while time.time() - start < 0.2:
            rx = bus.recv(0.01)
            if rx and rx.arbitration_id == 0x580+NODE_ID:
                return int.from_bytes(rx.data[4:8], 'little', signed=True)
        return None

    # 1. 确保在 PP 模式
    sdo_write(0x6060, 0, 1, 1)
    
    # 2. 正常使能
    print("[Action] Enabling...")
    for val in [0x06, 0x07, 0x0F]:
        sdo_write(0x6040, 0, val, 2)
        time.sleep(0.2)

    # 3. 强制给一个很大的位置偏差 (10 圈)
    print("[Action] Setting 80000 counts offset...")
    sdo_write(0x607A, 0, 80000, 4)
    sdo_write(0x6040, 0, 0x3F, 2) # Abs Move

    print("\n--- Monitoring Internal Values (10s) ---")
    print("If 'TorqueActual' is near 0, the position loop is NOT outputting torque.")
    
    start = time.time()
    try:
        while time.time() - start < 10:
            pos = sdo_read(0x6064, 0)
            # 读取 0x6077 (实际转矩/Iq电流反馈)
            torque = sdo_read(0x6077, 0)
            # 读取状态字
            sw = sdo_read(0x6041, 0)
            
            print(f"  Pos: {pos:6d} | Torque(Iq): {torque:6d} | SW: {hex(sw if sw else 0)}", end='\r')
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    print("\n\nDiagnosis Finished.")
    sdo_write(0x6040, 0, 0x06, 2)
    bus.shutdown()

if __name__ == "__main__":
    torque_diagnosis()
