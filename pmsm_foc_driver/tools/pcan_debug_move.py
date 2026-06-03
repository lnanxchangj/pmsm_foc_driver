import can
import time

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

def debug_pcan_control():
    print(f"--- CiA 402 Debug Control (Speed Up & Monitor) ---")
    try:
        bus = can.interface.Bus(interface='pcan', channel=CHANNEL, bitrate=BITRATE)
        print(f"Connected to {CHANNEL} @ {BITRATE}")
    except Exception as e:
        print(f"Error: {e}")
        return

    def sdo_write(index, subindex, data, length):
        if length == 4: cmd = SDO_WRITE_4BYTE
        elif length == 2: cmd = SDO_WRITE_2BYTE
        else: cmd = SDO_WRITE_1BYTE
        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]
        payload += list(data.to_bytes(4, 'little', signed=(data < 0)))
        bus.send(can.Message(arbitration_id=TX_SDO, data=payload, is_extended_id=False))
        start = time.time()
        while time.time() - start < 0.5:
            rx = bus.recv(0.01)
            if rx and rx.arbitration_id == RX_SDO and rx.data[0] == 0x60: return True
        return False

    def sdo_read(index, subindex):
        bus.send(can.Message(arbitration_id=TX_SDO, data=[SDO_READ, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0], is_extended_id=False))
        start = time.time()
        while time.time() - start < 0.5:
            rx = bus.recv(0.01)
            if rx and rx.arbitration_id == RX_SDO:
                return int.from_bytes(rx.data[4:8], 'little', signed=True)
        return None

    # 1. 配置
    print("Configuring Mode (PP) and high velocity (80000 counts/s)...")
    sdo_write(0x6060, 0, 1, 1)      # PP Mode
    sdo_write(0x6081, 0, 80000, 4)  # 10 RPS

    # 2. 使能
    print("Transitioning to OPERATION_ENABLED...")
    for val in [0x06, 0x07, 0x0F]:
        sdo_write(0x6040, 0, val, 2)
        time.sleep(0.1)
    
    sw = sdo_read(0x6041, 0)
    if sw and (sw & 0x006F) == 0x27:
        print("Status: OPERATION_ENABLED")
    else:
        print(f"Warning: Unexpected statusword: {hex(sw if sw else 0)}")

    initial_pos = sdo_read(0x6064, 0)
    print(f"Current Position: {initial_pos}")

    def move_and_monitor(target, relative=False):
        mode_str = "REL" if relative else "ABS"
        print(f"\nMoving {mode_str} to {target}...")
        sdo_write(0x607A, 0, int(target), 4)
        
        # bit 4: New Setpoint, bit 5: Change Immed, bit 6: Rel
        ctrl = 0x3F if relative else 0x1F
        sdo_write(0x6040, 0, ctrl, 2)
        
        start = time.time()
        reached = False
        while time.time() - start < 10.0:
            cur_sw = sdo_read(0x6041, 0)
            cur_pos = sdo_read(0x6064, 0)
            print(f"  SW: {hex(cur_sw if cur_sw else 0)} | Pos: {cur_pos}", end='\r')
            
            if cur_sw and (cur_sw & 0x1000): # Setpoint Ack
                sdo_write(0x6040, 0, 0x0F, 2) # Clear New Setpoint
                
            if cur_sw and (cur_sw & 0x0400): # Target Reached
                print(f"\n  Success: Target {target} reached at Pos: {cur_pos}")
                reached = True
                break
            time.sleep(0.1)
        
        if not reached: print("\n  Timeout waiting for Target Reached!")
        return reached

    # 执行序列
    move_and_monitor(0, relative=False)
    move_and_monitor(8000, relative=True)
    move_and_monitor(-4000, relative=True)
    move_and_monitor(8000, relative=True)
    move_and_monitor(0, relative=False)

    print("\nShutting down...")
    sdo_write(0x6040, 0, 0x06, 2)
    bus.shutdown()

if __name__ == "__main__":
    debug_pcan_control()
