import can
import time
import struct

# 配置参数
CHANNEL = 'PCAN_USBBUS1'
BITRATE = 500000
NODE_ID = 1

# SDO 命令头
SDO_WRITE_4BYTE = 0x23
SDO_WRITE_2BYTE = 0x2B
SDO_WRITE_1BYTE = 0x2F
SDO_READ        = 0x40
SDO_RESPONSE    = 0x60

# COB-ID
TX_SDO = 0x600 + NODE_ID
RX_SDO = 0x580 + NODE_ID

def raw_pcan_control():
    print(f"--- Raw PCAN CiA 402 Control ---")
    try:
        bus = can.interface.Bus(interface='pcan', channel=CHANNEL, bitrate=BITRATE)
        print(f"Connected to {CHANNEL} @ {BITRATE}")
    except Exception as e:
        print(f"Error connecting to PCAN: {e}")
        return

    def sdo_write(index, subindex, data, length):
        if length == 4: cmd = SDO_WRITE_4BYTE
        elif length == 2: cmd = SDO_WRITE_2BYTE
        else: cmd = SDO_WRITE_1BYTE
        
        # 组包: [Cmd, Index_Low, Index_High, SubIndex, Data0, Data1, Data2, Data3]
        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]
        payload += list(data.to_bytes(4, 'little', signed=(data < 0)))
        
        msg = can.Message(arbitration_id=TX_SDO, data=payload, is_extended_id=False)
        bus.send(msg)
        
        # 等待响应
        start = time.time()
        while time.time() - start < 1.0:
            rx = bus.recv(0.1)
            if rx and rx.arbitration_id == RX_SDO and rx.data[0] == 0x60:
                return True
        return False

    def sdo_read(index, subindex):
        payload = [SDO_READ, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        msg = can.Message(arbitration_id=TX_SDO, data=payload, is_extended_id=False)
        bus.send(msg)
        
        start = time.time()
        while time.time() - start < 1.0:
            rx = bus.recv(0.1)
            if rx and rx.arbitration_id == RX_SDO:
                # 简单解析 1/2/4 字节数据
                return int.from_bytes(rx.data[4:8], 'little')
        return None

    def wait_status(mask, val, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            sw = sdo_read(0x6041, 0)
            if sw is not None and (sw & mask) == val:
                return sw
            time.sleep(0.05)
        return None

    # 1. 设置 PP 模式
    print("Setting Mode of Operation to PP (1)...")
    sdo_write(0x6060, 0, 1, 1)

    # 2. 状态机切换到 Operation Enabled
    print("Transitioning State Machine...")
    sdo_write(0x6040, 0, 0x06, 2) # Shutdown
    time.sleep(0.1)
    sdo_write(0x6040, 0, 0x07, 2) # Switch On
    time.sleep(0.1)
    sdo_write(0x6040, 0, 0x0F, 2) # Enable Operation
    
    if wait_status(0x006F, 0x0027):
        print("Status: OPERATION_ENABLED")
    else:
        print("Failed to reach Operation Enabled. Check Power/Faults.")
        # return

    COUNTS_PER_REV = 8000

    def move(target, relative=False):
        mode_str = "REL" if relative else "ABS"
        print(f"Move {mode_str}: {target}")
        
        # 写入目标位置 0x607A
        sdo_write(0x607A, 0, int(target), 4)
        
        # 控制字: 
        # bit 4: New Setpoint
        # bit 5: Change Set Immediately
        # bit 6: 0=Abs, 1=Rel
        ctrl = 0x3F if relative else 0x1F
        sdo_write(0x6040, 0, ctrl, 2)
        
        # 等待 Ack
        if wait_status(0x1000, 0x1000):
            sdo_write(0x6040, 0, 0x0F, 2) # Clear New Setpoint
            print("Moving...")
            wait_status(0x0400, 0x0400, timeout=10.0)
            print("Target Reached.")
        else:
            print("Move timeout (Ack not received)")

    # 执行序列
    move(0, relative=False)              # 归零
    move(COUNTS_PER_REV, relative=True)  # 正向一圈
    move(-COUNTS_PER_REV//2, relative=True) # 反向半圈
    move(COUNTS_PER_REV, relative=True)  # 正向一圈
    move(0, relative=False)              # 归零

    print("Sequence Complete. Shutting down...")
    sdo_write(0x6040, 0, 0x06, 2)
    bus.shutdown()

if __name__ == "__main__":
    raw_pcan_control()
