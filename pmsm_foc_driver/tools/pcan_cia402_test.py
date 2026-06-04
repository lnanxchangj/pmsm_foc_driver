import canopen
import time
import threading
import serial

# PCAN 通信参数
PCAN_INTERFACE = 'pcan'
PCAN_CHANNEL   = 'PCAN_USBBUS1'
BITRATE        = 500000
NODE_ID        = 1
EDS_PATH       = r'CANopen\CANopenNode_STM32\DS301_profile.eds'

# 串口参数
SERIAL_PORT = 'COM6'
SERIAL_BAUD = 115200

def read_serial_log():
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"--- 串口 {SERIAL_PORT} 已打开 ---")
        while True:
            line = ser.readline()
            if line:
                try:
                    print(f"[COM4] {line.decode('utf-8', errors='ignore').strip()}")
                except:
                    pass
    except Exception as e:
        print(f"无法打开串口 {SERIAL_PORT}，如果被占用或者不存在请忽略: {e}")

def main():
    print(f"--- CiA 402 测试 (PCAN + COM4 日志) ---")
    
    # 启动串口读取线程
    serial_thread = threading.Thread(target=read_serial_log, daemon=True)
    serial_thread.start()

    # 1. 创建并连接网络
    network = canopen.Network()
    try:
        network.connect(interface=PCAN_INTERFACE, channel=PCAN_CHANNEL, bitrate=BITRATE)
        print(f"已连接到 {PCAN_INTERFACE} {PCAN_CHANNEL} ({BITRATE}bps)")
    except Exception as e:
        print(f"PCAN 连接失败: {e}")
        print("请检查 PCAN 驱动和连接状态。")
        return

    # 2. 添加节点
    try:
        node = network.add_node(NODE_ID, EDS_PATH)
        print(f"节点 {NODE_ID} 已添加")
    except Exception as e:
        print(f"添加节点失败: {e}")
        return

    # 3. 将节点设置为 Operational
    node.nmt.state = 'OPERATIONAL'
    time.sleep(0.5)

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
        print(f"正在切换状态至 {target}...")
        for _ in range(20): 
            sw = get_sw()
            if (sw & 0x0008): # Fault
                set_cw(0x80) # Fault Reset
            elif (sw & 0x004F) == 0x0040: # Switch On Disabled
                set_cw(0x06) # Shutdown
            elif (sw & 0x006F) == 0x0021: # Ready to Switch On
                set_cw(0x07) # Switch On
            elif (sw & 0x006F) == 0x0023: # Switched On
                set_cw(0x0F) # Enable Operation
            
            if target == "OPERATION_ENABLED" and (sw & 0x006F) == 0x27:
                print(">>> 电机已使能 (OPERATION_ENABLED) <<<")
                return True
            time.sleep(0.2)
        return False

    # ========== 回零模式 (HM) 测试 ==========
    print("\n[测试 1] 切换到 HM 模式 (Homing)")
    set_cw(0x06) # Shutdown
    time.sleep(0.5)
    node.sdo.download(0x6060, 0, b'\x06') # 6 = HM Mode
    time.sleep(0.1)

    print("\n[测试 1.1] 设置回零参数")
    # 寻零位 (33 = 0x21)
    node.sdo.download(0x6098, 0, b'\x21')
    # 设置回零速度 (6099 sub 1/2) 为 60 RPM
    node.sdo.download(0x6099, 1, int(60).to_bytes(4, 'little', signed=False))
    node.sdo.download(0x6099, 2, int(60).to_bytes(4, 'little', signed=False))

    if not set_cia402_state("OPERATION_ENABLED"):
        print("使能失败，退出。")
        return

    print("\n[测试 1.2] 触发回零 (Homing Operation Start)")
    set_cw(0x1F) # Enable Op (0x0F) + Homing Start (0x10)
    
    # 等待回零完成 (Bit 12 Homing Attained = 1, Bit 10 Target Reached = 1)
    # Mask = 0x1400, Value = 0x1400
    if wait_status(0x1400, 0x1400, timeout=10.0):
        print(">>> 回零成功完成 (Homing Attained) <<<")
    else:
        print(">>> 回零超时或失败 <<<")
    
    set_cw(0x0F) # 清除 Homing Start
    time.sleep(1.0)


    # ========== 位置模式 (PP) 测试 ==========
    print("\n[测试 2] 切换到 PP 模式 (Profile Position)")
    set_cw(0x06) # 切换模式前先关闭使能
    time.sleep(0.5)
    node.sdo.download(0x6060, 0, b'\x01') # 1 = PP Mode
    time.sleep(0.1)

    print("\n[测试 2.1] 设置运行参数")
    # 设置运动速度为 100 RPM
    node.sdo.download(0x6081, 0, int(60).to_bytes(4, 'little', signed=False))
    print("已设置 Profile Velocity (0x6081) = 60 RPM")

    if not set_cia402_state("OPERATION_ENABLED"):
        print("使能失败，退出。")
        return

    COUNTS_PER_REV = 8000
    
    def do_pp_move(target, relative=False):
        flags = 0x3F # Enable Op (0x0F) + New Setpoint (0x10) + Change Immed (0x20)
        if relative:
            flags |= 0x40 
            print(f"相对运动: {target} counts")
        else:
            print(f"绝对运动: {target} counts")

        try:
            node.sdo.download(0x607A, 0, int(target).to_bytes(4, 'little', signed=True))
            set_cw(flags)
            wait_status(0x1000, 0x1000) # Wait Setpoint Ack
            set_cw(flags & ~0x10) # Clear New Setpoint
            wait_status(0x0400, 0x0400, timeout=10.0) # Wait Target Reached
            print(">>> 运动完成 <<<")
        except Exception as e:
            print(f"运动错误: {e}")

    print("\n[测试 2.2] 绝对运动到 0.5 圈位置 (4000 counts)")
    do_pp_move(4000, relative=False)
    time.sleep(1)

    print("\n[测试 2.3] 绝对运动到 45° 位置 (1000 counts)")
    do_pp_move(1000, relative=False)
    time.sleep(1)

    print("\n[测试 2.4] 绝对运动回到 0 零点位置 (0 counts)")
    do_pp_move(0, relative=False)
    time.sleep(1)

    print("\n--- 测试完成，断开连接 ---")
    try:
        set_cw(0x06) # Shutdown
    except:
        pass
    network.disconnect()

if __name__ == "__main__":
    main()
