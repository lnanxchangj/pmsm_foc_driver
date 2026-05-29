import can
import time
import struct

def sdo_write(bus, node_id, index, subindex, value, size=4):
    tx_id = 0x600 + node_id
    rx_id = 0x580 + node_id
    if size == 4:
        cmd = 0x23
        payload = struct.pack('<I', value & 0xFFFFFFFF)
    elif size == 2:
        cmd = 0x2B
        payload = struct.pack('<H', value & 0xFFFF) + b'\x00\x00'
    elif size == 1:
        cmd = 0x2F
        payload = struct.pack('<B', value & 0xFF) + b'\x00\x00\x00'
    
    data = bytes([cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]) + payload
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=False))
    
    deadline = time.time() + 1.0
    while time.time() < deadline:
        msg = bus.recv(0.1)
        if msg and msg.arbitration_id == rx_id:
            return True
    return False

def sdo_read(bus, node_id, index, subindex=0):
    tx_id = 0x600 + node_id
    rx_id = 0x580 + node_id
    data = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=False))
    deadline = time.time() + 1.0
    while time.time() < deadline:
        msg = bus.recv(0.1)
        if msg and msg.arbitration_id == rx_id:
            return msg.data
    return None

def main():
    node_id = 0x01
    bus = can.interface.Bus(interface="pcan", bitrate=500000, channel="PCAN_USBBUS1")
    
    print("Setting mode to PP (Profile Position)...")
    sdo_write(bus, node_id, 0x6060, 0, 1, size=1)
    time.sleep(0.1)
    
    # Verify mode
    data = sdo_read(bus, node_id, 0x6061)
    if data:
        print(f"Current Mode Display: {data[4]}")
    
    # Get current position
    data = sdo_read(bus, node_id, 0x6064)
    if data:
        current_pos = struct.unpack('<i', data[4:8])[0]
        print(f"Current Position: {current_pos}")
    else:
        current_pos = 0

    # Move relative 1000 units (approx 90 deg or 141 deg depending on scale)
    target_pos = current_pos + 1000
    print(f"Moving to target position: {target_pos} (Relative +1000)")
    
    # We use Absolute movement for now by just writing the target
    # Note: In PP mode, x607A is the target. 
    # The firmware detects changes to x607A to trigger movement.
    sdo_write(bus, node_id, 0x607A, 0, target_pos, size=4)
    
    print("Waiting for movement...")
    for _ in range(20):
        time.sleep(0.5)
        data_sw = sdo_read(bus, node_id, 0x6041)
        data_pos = sdo_read(bus, node_id, 0x6064)
        if data_sw and data_pos:
            sw = struct.unpack('<H', data_sw[4:6])[0]
            pos = struct.unpack('<i', data_pos[4:8])[0]
            target_reached = (sw & (1 << 10)) != 0
            print(f"Statusword: 0x{sw:04X}, Position: {pos}, Reached: {target_reached}")
            if target_reached:
                print("Target reached!")
                break
    
    bus.shutdown()

if __name__ == "__main__":
    main()
