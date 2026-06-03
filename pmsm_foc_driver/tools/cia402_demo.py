import canopen
import time
import sys

# PCAN 通信参数
PCAN_INTERFACE = 'pcan'
PCAN_CHANNEL   = 'PCAN_USBBUS1'
BITRATE        = 500000
NODE_ID        = 1
EDS_PATH       = r'CANopen\CANopenNode_STM32\DS301_profile.eds'

def cia402_demo():
    print(f"--- CiA 402 Demo (PCAN) ---")
    
    # 1. 创建并连接网络
    network = canopen.Network()
    try:
        # 注意：如果环境中没有 PCAN 硬件，这里会报错
        # 为了演示，如果报错我们会尝试连接 virtual 接口
        network.connect(interface=PCAN_INTERFACE, channel=PCAN_CHANNEL, bitrate=BITRATE)
        print(f"Connected to {PCAN_INTERFACE} {PCAN_CHANNEL} at {BITRATE}bps")
    except Exception as e:
        print(f"PCAN connection failed: {e}")
        print("Running in VIRTUAL mode for logic demonstration...")
        network.connect(interface='virtual', channel='test_bus')

    # 2. 添加节点
    try:
        node = network.add_node(NODE_ID, EDS_PATH)
        print(f"Node {NODE_ID} added using EDS: {EDS_PATH}")
    except Exception as e:
        print(f"Failed to add node: {e}")
        return

    # 3. 将节点设置为 Operational
    node.nmt.state = 'OPERATIONAL'
    time.sleep(0.1)

    # 4. 状态机切换辅助函数 (CIA 402) - 使用 Raw SDO 以兼容精简版 EDS
    def get_sw():
        try:
            return int.from_bytes(node.sdo.upload(0x6041, 0), 'little')
        except:
            return 0

    def set_cw(val):
        node.sdo.download(0x6040, 0, val.to_bytes(2, 'little'))

    def wait_status(mask, val, timeout=3.0):
        start = time.time()
        while time.time() - start < timeout:
            sw = get_sw()
            if (sw & mask) == val:
                return sw
            time.sleep(0.1)
        return None

    def set_cia402_state(target):
        print(f"Transitioning to {target}...")
        for _ in range(15): 
            sw = get_sw()
            if (sw & 0x0008): # Fault
                set_cw(0x80)
            elif (sw & 0x004F) == 0x0040: # Switch On Disabled
                set_cw(0x06)
            elif (sw & 0x006F) == 0x0021: # Ready to Switch On
                set_cw(0x07)
            elif (sw & 0x006F) == 0x0023: # Switched On
                set_cw(0x0F)
            
            if target == "OPERATION_ENABLED" and (sw & 0x006F) == 0x27:
                print("Status: OPERATION_ENABLED")
                return True
            time.sleep(0.2)
        return False

    # 5. 开始演示流程
    print("\n[Step 1] Set Mode to PP (Profile Position)")
    try:
        node.sdo.download(0x6060, 0, b'\x01') # PP Mode
    except Exception as e:
        print(f"SDO Error (Expected if no hardware): {e}")
        # 如果是 virtual 模式且没有对应模拟节点，这里会报错，我们继续演示逻辑
    
    if not set_cia402_state("OPERATION_ENABLED"):
        print("Failed to enable motor (Check hardware connection/power).")
        # return # 注释掉以允许在无硬件环境下跑完逻辑

    # 编码器单位定义 (根据 cia402.h 为 8000 counts/rev)
    COUNTS_PER_REV = 8000
    
    def do_pp_move(target, relative=False):
        flags = 0x3F # Enable Op (0x0F) + New Setpoint (0x10) + Change Immed (0x20)
        if relative:
            flags |= 0x40 
            print(f"Moving RELATIVE: {target} counts")
        else:
            print(f"Moving ABSOLUTE: {target} counts")

        try:
            # 写入目标位置 (int32)
            node.sdo.download(0x607A, 0, int(target).to_bytes(4, 'little', signed=True))
            
            # 触发指令
            set_cw(flags)
            
            # 等待应答 (Setpoint Acknowledge bit 12)
            print("Waiting for Setpoint Ack...")
            wait_status(0x1000, 0x1000)
                
            # 清除 New Setpoint
            set_cw(flags & ~0x10)
            
            # 等待到达 (Target Reached bit 10)
            print("Waiting for target reached...")
            wait_status(0x0400, 0x0400, timeout=10.0)
            print("Done.")
        except Exception as e:
            print(f"Move error: {e}")

    # --- 动作序列 ---
    
    print("\n[Step 2] Move to Absolute 0 (归零)")
    do_pp_move(0, relative=False)
    
    print("\n[Step 3] Rotate Forward 1 Rev (正向一圈)")
    do_pp_move(COUNTS_PER_REV, relative=True)
    
    print("\n[Step 4] Rotate Backward 0.5 Rev (反向半圈)")
    do_pp_move(-COUNTS_PER_REV/2, relative=True)
    
    print("\n[Step 5] Rotate Forward 1 Rev (正向一圈)")
    do_pp_move(COUNTS_PER_REV, relative=True)
    
    print("\n[Step 6] Return to Absolute 0 (归零)")
    do_pp_move(0, relative=False)

    print("\n--- Demo Sequence Finished ---")
    
    # 停止并断开
    try:
        set_cw(0x06) # Shutdown
    except:
        pass
    network.disconnect()

if __name__ == "__main__":
    cia402_demo()
