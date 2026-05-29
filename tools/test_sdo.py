import can
import time

bus = can.interface.Bus(interface="pcan", bitrate=500000, channel="PCAN_USBBUS1")

def test_sdo():
    print("Flushing bus...")
    while bus.recv(0.1):
        pass

    # Read Statusword 0x6041 sub 0
    # CCS=2 (Initiate Upload) -> 0x40
    data = bytes([0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
    msg = can.Message(arbitration_id=0x601, data=data, is_extended_id=False)
    
    print(f"Sending SDO request to 0x601: {[hex(x) for x in data]}")
    bus.send(msg)

    print("Waiting for response...")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        msg = bus.recv(0.1)
        if msg:
            print(f"Received ID: {hex(msg.arbitration_id)} Data: {[hex(x) for x in msg.data]}")
            if msg.arbitration_id == 0x581:
                print("Got SDO response!")
                return
    print("Timeout!")

test_sdo()
bus.shutdown()
