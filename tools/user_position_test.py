import struct, time, sys, math, can
NODE_ID = 0x01
RPDO1_ID = 0x200 + NODE_ID
SDO_TX_ID = 0x600 + NODE_ID
SDO_RX_ID = 0x580 + NODE_ID
def can_init():
    return can.interface.Bus(interface='pcan', bitrate=500000, channel='PCAN_USBBUS1')
def sdo_write(bus, index, subindex, value, size=4):
    n = {4: 0, 3: 1, 2: 2, 1: 3}[size]
    cmd = 0x20 | (n << 2) | 0x02 | 0x01
    payload = struct.pack('<I', value & 0xFFFFFFFF)
    data = bytes([cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]) + payload[:4]
    bus.send(can.Message(arbitration_id=SDO_TX_ID, data=data, is_extended_id=False))
    start_time = time.time()
    while time.time() - start_time < 0.5:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == SDO_RX_ID and msg.data[1] == (index & 0xFF):
            return msg.data[0] == 0x60
    return False
def sdo_read(bus, index, subindex=0):
    data = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
    msg = can.Message(arbitration_id=SDO_TX_ID, data=data, is_extended_id=False)
    bus.send(msg)
    start_time = time.time()
    while time.time() - start_time < 0.5:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == SDO_RX_ID and msg.data[1] == (index & 0xFF):
            if msg.data[0] & 0x40:
                return struct.unpack_from('<i', msg.data, 4)[0]
    return None
def send_cw(bus, cw):
    data = struct.pack('<H', cw) + b'\x00\x00\x00\x00'
    msg = can.Message(arbitration_id=RPDO1_ID, data=data, is_extended_id=False)
    bus.send(msg)
def wait_tr(bus, label, timeout=10.0):
    print('--- Polling %s ---' % label)
    start_time = time.time()
    while time.time() - start_time < timeout:
        sw = sdo_read(bus, 0x6041)
        pos = sdo_read(bus, 0x6064)
        if sw is not None:
            print('  SW=0x%04X Pos=%d' % (sw, pos if pos is not None else 0))
            if sw & (1 << 10):
                print('--- Target Reached ---')
                return True
        time.sleep(0.5)
    print('--- Timeout ---')
    return False
def perform_move(bus, target, is_rel, velocity_rpm, label):
    print('Executing: %s (Target=%d, Rel=%s, Vel=%d RPM)' % (label, target, is_rel, velocity_rpm))
    # NEW UNITS: 0x6081 is counts/s
    vel_counts_s = int(velocity_rpm * 4000 / 60)
    sdo_write(bus, 0x6081, 0, vel_counts_s)
    sdo_write(bus, 0x607A, 0, target & 0xFFFFFFFF)
    cw = 0x000F | (0x0040 if is_rel else 0x0000)
    send_cw(bus, cw | 0x0010)
    time.sleep(0.1)
    send_cw(bus, cw)
    return wait_tr(bus, label)
def main():
    try:
        bus = can_init()
        sdo_write(bus, 0x6060, 0, 1, size=1)
        send_cw(bus, 0x0006); time.sleep(0.1)
        send_cw(bus, 0x0007); time.sleep(0.1)
        send_cw(bus, 0x000F); time.sleep(1.0)
        perform_move(bus, 8000, False, 300, 'Move 1: To 2 Turns (8000)')
        print('Wait 5s...')
        time.sleep(5.0)
        perform_move(bus, 4000, False, 600, 'Move 2: To 1 Turn (4000)')
        perform_move(bus, 5000, False, 150, 'Move 3: To 1.25 Turns (5000)')
        send_cw(bus, 0x0000)
        bus.shutdown()
    except Exception as e:
        print('Error: %s' % e)
main()
