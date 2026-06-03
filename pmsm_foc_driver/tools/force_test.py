import can
import time

def force_test():
    try:
        bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
    except Exception as e:
        print(f"Connection error: {e}")
        return
        
    NODE_ID = 1
    
    def sdo_w(idx, sub, val, size):
        payload = [0x23 if size==4 else (0x2B if size==2 else 0x2F), idx&0xFF, idx>>8, sub]
        payload += list(int(val).to_bytes(4, 'little', signed=(val < 0)))
        bus.send(can.Message(arbitration_id=0x600+NODE_ID, data=payload, is_extended_id=False))
        time.sleep(0.1)

    print("\n--- Force Movement Test ---")
    
    # 1. 速度模式测试 (500 RPM)
    print("[1/2] Testing Velocity Mode (PV)...")
    sdo_w(0x6060, 0, 3, 1) # PV Mode
    for v in [0x06, 0x07, 0x0F]: sdo_w(0x6040, 0, v, 2)
    sdo_w(0x60FF, 0, 500, 4) # 500 RPM
    
    start = time.time()
    while time.time() - start < 3.0:
        bus.send(can.Message(arbitration_id=0x601, data=[0x40, 0x6C, 0x60, 0x00, 0,0,0,0])) # Actual Vel
        r = bus.recv(0.05)
        vel = int.from_bytes(r.data[4:8], 'little', signed=True) if r and r.arbitration_id==0x581 else 0
        print(f"  Speed: {vel:8d} RPM", end='\r')
        time.sleep(0.1)
    print("\n  PV Test Ended.")

    # 2. 切换到位置模式 (PP)
    print("\n[2/2] Testing Position Mode (PP)...")
    sdo_w(0x6060, 0, 1, 1) # PP Mode
    sdo_w(0x6081, 0, 40000, 4) # 5 RPS
    
    # 触发一个超大位移 (20 圈)
    print("  Triggering 160,000 counts move...")
    sdo_w(0x607A, 0, 160000, 4) 
    sdo_w(0x6040, 0, 0x3F, 2) # Abs Move Trigger
    
    print("  Monitoring Torque and Position...")
    start = time.time()
    while time.time() - start < 6.0:
        # 读取 0x6064 (位置)
        bus.send(can.Message(arbitration_id=0x601, data=[0x40, 0x64, 0x60, 0x00, 0,0,0,0]))
        r1 = bus.recv(0.05)
        # 读取 0x6077 (转矩/电流)
        bus.send(can.Message(arbitration_id=0x601, data=[0x40, 0x77, 0x60, 0x00, 0,0,0,0]))
        r2 = bus.recv(0.05)
        
        p = int.from_bytes(r1.data[4:8], 'little', signed=True) if r1 and r1.arbitration_id==0x581 else 0
        t = int.from_bytes(r2.data[4:8], 'little', signed=True) if r2 and r2.arbitration_id==0x581 else 0
        print(f"  Pos: {p:8d} | Torque: {t:8d}", end='\r')
        time.sleep(0.1)

    print("\n\nTest Finished. Disabling...")
    sdo_w(0x6040, 0, 0x06, 2)
    bus.shutdown()

if __name__ == "__main__":
    force_test()
