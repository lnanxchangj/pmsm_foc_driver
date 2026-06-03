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
SERIAL_PORT = 'COM4'
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

    # ========== 速度模式 (PV) 测试 ==========
    # print("\n[测试 1] 切换到 PV 模式 (Profile Velocity) - 已跳过")
    # node.sdo.download(0x6060, 0, b'\x03') # 3 = PV Mode
    # time.sleep(0.1)
    # 
    # if not set_cia402_state("OPERATION_ENABLED"):
    #     print("使能失败，退出。")
    #     return
    #
    # # 设定速度为 4000 rpm
    # target_vel = 4000
    # print(f"\n[测试 1.1] 设定目标速度: {target_vel} RPM")
    # node.sdo.download(0x60FF, 0, int(target_vel).to_bytes(4, 'little', signed=True))
    # time.sleep(5.0)
    #
    # # 设定速度为 0 rpm
    # target_vel = 0
    # print(f"\n[测试 1.2] 设定目标速度: {target_vel} RPM (停止)")
    # node.sdo.download(0x60FF, 0, int(target_vel).to_bytes(4, 'little', signed=True))
    # time.sleep(2.0)

    # ========== 位置模式 (PP) 测试 ==========
    print("\n[测试 2] 切换到 PP 模式 (Profile Position)")
    # 切换模式前先关闭使能
    set_cw(0x06)
    time.sleep(0.5)
    node.sdo.download(0x6060, 0, b'\x01') # 1 = PP Mode
    time.sleep(0.1)
    
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

    print("\n[测试 2.1] 相对运动 1 圈 (8000 counts)")
    do_pp_move(COUNTS_PER_REV, relative=True)
    time.sleep(1)

    print("\n[测试 2.2] 相对运动 -1 圈 (-8000 counts)")
    do_pp_move(-COUNTS_PER_REV, relative=True)
    time.sleep(1)

    print("\n--- 测试完成，断开连接 ---")
    try:
        set_cw(0x06) # Shutdown
    except:
        pass
    network.disconnect()

if __name__ == "__main__":
    main()
