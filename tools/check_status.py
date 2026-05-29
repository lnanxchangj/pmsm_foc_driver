import can
import time
import struct

def read_sdo(bus, node_id, index, subindex=0):
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
    
    # Read Mode (0x6061)
    data = read_sdo(bus, node_id, 0x6061)
    if data:
        mode = data[4]
        print(f"Current Mode (0x6061): {mode}")
    
    # Read Position (0x6064)
    data = read_sdo(bus, node_id, 0x6064)
    if data:
        pos = struct.unpack('<i', data[4:8])[0]
        print(f"Current Position (0x6064): {pos}")
        
    bus.shutdown()

if __name__ == "__main__":
    main()
