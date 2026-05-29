import can
import time
import struct

def check_node(node_id):
    print(f"Checking Node 0x{node_id:02X} (16)...")
    try:
        bus = can.interface.Bus(interface="pcan", bitrate=500000, channel="PCAN_USBBUS1")
    except Exception as e:
        print(f"Error opening PCAN: {e}")
        return

    # Flush
    while bus.recv(0.01): pass

    # Read Statusword 0x6041
    tx_id = 0x600 + node_id
    rx_id = 0x580 + node_id
    data = [0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=False))

    deadline = time.time() + 1.0
    while time.time() < deadline:
        msg = bus.recv(0.1)
        if msg and msg.arbitration_id == rx_id:
            sw = struct.unpack('<H', msg.data[4:6])[0]
            print(f"SUCCESS: Node 0x{node_id:02X} responded. Statusword: 0x{sw:04X}")
            bus.shutdown()
            return True
    
    print(f"FAILED: No response from Node 0x{node_id:02X}")
    bus.shutdown()
    return False

if __name__ == "__main__":
    check_node(0x10)
