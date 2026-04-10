#!/usr/bin/env python3
"""
CiA 402 CANopen Test Tool for PMSM FOC Driver
Uses PCAN USB-CAN adapter to communicate with STM32 node.

CAN Configuration:
  - Node ID: 16 (0x10)
  - Baudrate: 500 kbps
  - RPDO1 (0x210): Controlword(2B) + TargetVelocity(4B)
  - TPDO1 (0x190): Statusword(2B) + VelocityActual(4B)
  - SDO TX: 0x610 / SDO RX: 0x590

Usage:
  pip install -r requirements.txt
  python cia402_test.py
"""

import struct
import time
import sys
import can

# ===========================================================================
# Configuration
# ===========================================================================
NODE_ID = 0x10
BITRATE = 500000

# PDO COB IDs (with node ID offset)
RPDO1_ID = 0x200 + NODE_ID   # 0x210 - PC sends CW + TargetVelocity
TPDO1_ID = 0x180 + NODE_ID   # 0x190 - PC receives SW + VelocityActual

# SDO COB IDs
SDO_TX_ID = 0x600 + NODE_ID  # 0x610 - PC sends SDO request
SDO_RX_ID = 0x580 + NODE_ID  # 0x590 - PC receives SDO response

# ===========================================================================
# CiA 402 Controlword commands
# ===========================================================================
CW_SHUTDOWN          = 0x0006  # bits: 1=1, 2=1
CW_SWITCH_ON         = 0x0007  # bits: 0=1, 1=1, 2=1
CW_ENABLE_OPERATION  = 0x000F  # bits: 0=1, 1=1, 2=1, 3=1
CW_DISABLE_OPERATION = 0x0007  # bits: 0=1, 1=1, 2=1
CW_DISABLE_VOLTAGE   = 0x0000  # bits: 1=0
CW_QUICK_STOP        = 0x0002  # bits: 1=1, 2=0
CW_FAULT_RESET       = 0x0080  # bit7=1 (rising edge)
CW_NEW_SETPOINT      = 0x0010  # bit4=1 (PP mode)

# ===========================================================================
# CiA 402 Statusword bit masks
# ===========================================================================
SW_READY_TO_SWITCH_ON  = 1 << 0
SW_SWITCHED_ON         = 1 << 1
SW_OPERATION_ENABLED   = 1 << 2
SW_FAULT               = 1 << 3
SW_VOLTAGE_ENABLED     = 1 << 4
SW_QUICK_STOP          = 1 << 5
SW_SWITCH_ON_DISABLED  = 1 << 6
SW_WARNING             = 1 << 7
SW_REMOTE              = 1 << 9
SW_TARGET_REACHED      = 1 << 10
SW_INTERNAL_LIMIT      = 1 << 11
SW_SETPOINT_ACK        = 1 << 12

# ===========================================================================
# Operation modes
# ===========================================================================
MODE_NO_MODE = 0
MODE_PP = 1
MODE_PV = 3
MODE_PT = 4
MODE_HM = 6
MODE_CSP = 8
MODE_CSV = 9
MODE_CST = 10

MODE_NAMES = {
    MODE_NO_MODE: "No Mode",
    MODE_PP: "Profile Position",
    MODE_PV: "Profile Velocity",
    MODE_PT: "Profile Torque",
    MODE_HM: "Homing",
    MODE_CSP: "Cyclic Sync Position",
    MODE_CSV: "Cyclic Sync Velocity",
    MODE_CST: "Cyclic Sync Torque",
}

# ===========================================================================
# CAN Bus interface
# ===========================================================================
bus = None


def can_init():
    global bus
    try:
        bus = can.interface.Bus(bustype="pcan", bitrate=BITRATE)
        print(f"[OK] PCAN initialized, bitrate={BITRATE}")
    except Exception as e:
        print(f"[ERR] Failed to initialize PCAN: {e}")
        print("Make sure PCAN-USB driver is installed and device is connected.")
        sys.exit(1)


def can_send(cob_id, data):
    msg = can.Message(arbitration_id=cob_id, data=data, is_extended_id=False)
    bus.send(msg)


def can_recv(timeout_ms=1000):
    msg = bus.recv(timeout=timeout_ms / 1000.0)
    return msg


# ===========================================================================
# RPDO: Send Controlword + Target Velocity
# ===========================================================================
def send_rpdo1(controlword, target_velocity=0):
    """Send RPDO1: CW(2B LE) + TargetVelocity(4B LE) -> CAN ID 0x210"""
    data = struct.pack("<Hi", controlword, target_velocity)
    can_send(RPDO1_ID, data)


# ===========================================================================
# TPDO: Receive Statusword + Velocity Actual
# =========================================================================##
def recv_tpdo1(timeout_ms=2000):
    """Receive TPDO1 from CAN ID 0x190. Returns (statusword, velocity_actual) or None."""
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        msg = can_recv(timeout_ms=int((deadline - time.time()) * 1000))
        if msg is None:
            break
        if msg.arbitration_id == TPDO1_ID and len(msg.data) >= 6:
            sw = struct.unpack_from("<H", msg.data, 0)[0]
            vel = struct.unpack_from("<i", msg.data, 2)[0]
            return (sw, vel)
    return None


# ===========================================================================
# SDO: Expedited Read / Write
# ===========================================================================
def sdo_read(index, subindex=0, timeout_ms=2000):
    """
    Read an OD entry via SDO (expedited upload).
    Returns int value or None on error.
    """
    # CCS=2 (initiate upload): byte0 = 0x40
    data = bytes([0x40,
                  index & 0xFF, (index >> 8) & 0xFF,
                  subindex, 0, 0, 0, 0])
    can_send(SDO_TX_ID, data)

    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        msg = can_recv(timeout_ms=int((deadline - time.time()) * 1000))
        if msg is None:
            break
        if msg.arbitration_id == SDO_RX_ID and len(msg.data) >= 8:
            cmd = msg.data[0]
            rx_index = msg.data[1] | (msg.data[2] << 8)
            rx_sub = msg.data[3]

            if rx_index != index or rx_sub != subindex:
                continue

            # Check for SDO abort
            if (cmd & 0xE0) == 0x80:
                abort_code = struct.unpack_from("<I", msg.data, 4)[0]
                print(f"  SDO abort: index=0x{index:04X} sub={subindex} code=0x{abort_code:08X}")
                return None

            # Expedited upload response: CCS=2, S=1, E=1
            if (cmd & 0xE0) == 0x40:  # SCS=2 (upload response)
                s = (cmd >> 4) & 1
                e = (cmd >> 3) & 1
                if s and e:
                    n = (cmd >> 2) & 3  # bytes not used
                    raw = msg.data[4:8]
                    # Determine data size from n
                    valid_bytes = 4 - n
                    if valid_bytes <= 1:
                        return raw[0]
                    elif valid_bytes <= 2:
                        return struct.unpack_from("<H", raw)[0]
                    else:
                        return struct.unpack_from("<I", raw)[0]
                else:
                    # Segmented transfer not supported here
                    print("  SDO: segmented transfer not supported")
                    return None
    print(f"  SDO read timeout: index=0x{index:04X} sub={subindex}")
    return None


def sdo_write(index, subindex, value, data_size=4, timeout_ms=2000):
    """
    Write an OD entry via SDO (expedited download).
    data_size: 1, 2, or 4 bytes.
    Returns True on success.
    """
    n = {4: 0, 2: 2, 3: 1, 1: 3}[data_size]  # n = 4 - data_size
    cmd = 0x20 | (1 << 4) | (1 << 3) | (n << 2)  # CCS=1, S=1, E=1, n

    if data_size == 1:
        payload = struct.pack("<I", value & 0xFF)
    elif data_size == 2:
        payload = struct.pack("<I", value & 0xFFFF)
    else:
        payload = struct.pack("<I", value & 0xFFFFFFFF)

    data = bytes([cmd,
                  index & 0xFF, (index >> 8) & 0xFF,
                  subindex]) + payload
    can_send(SDO_TX_ID, data)

    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        msg = can_recv(timeout_ms=int((deadline - time.time()) * 1000))
        if msg is None:
            break
        if msg.arbitration_id == SDO_RX_ID and len(msg.data) >= 8:
            cmd = msg.data[0]
            rx_index = msg.data[1] | (msg.data[2] << 8)
            rx_sub = msg.data[3]

            if rx_index != index or rx_sub != subindex:
                continue

            # Abort
            if (cmd & 0xE0) == 0x80:
                abort_code = struct.unpack_from("<I", msg.data, 4)[0]
                print(f"  SDO abort: index=0x{index:04X} sub={subindex} code=0x{abort_code:08X}")
                return False

            # Download confirm: CCS=3 (SCS=1)
            if (cmd & 0xE0) == 0x20:
                return True
    print(f"  SDO write timeout: index=0x{index:04X} sub={subindex}")
    return False


# ===========================================================================
# Helpers
# ===========================================================================
def decode_state(sw):
    """Decode statusword into human-readable state name."""
    if (sw & 0x006F) == 0x0000:
        return "NOT_READY_TO_SWITCH_ON"
    elif (sw & 0x006F) == 0x0040:
        return "SWITCH_ON_DISABLED"
    elif (sw & 0x006F) == 0x0021:
        return "READY_TO_SWITCH_ON"
    elif (sw & 0x006F) == 0x0023:
        return "SWITCHED_ON"
    elif (sw & 0x006F) == 0x0027:
        return "OPERATION_ENABLED"
    elif (sw & 0x006F) == 0x0007:
        return "QUICK_STOP_ACTIVE"
    elif (sw & 0x006F) == 0x000F:
        return "FAULT_REACTION_ACTIVE"
    elif (sw & 0x006F) == 0x0008:
        return "FAULT"
    else:
        return f"UNKNOWN(0x{sw:04X})"


def print_statusword(sw):
    state = decode_state(sw)
    flags = []
    if sw & SW_VOLTAGE_ENABLED:
        flags.append("VoltEn")
    if sw & SW_QUICK_STOP:
        flags.append("QStop=normal")
    else:
        flags.append("QStop=ACTIVE")
    if sw & SW_WARNING:
        flags.append("WARNING")
    if sw & SW_REMOTE:
        flags.append("Remote")
    if sw & SW_TARGET_REACHED:
        flags.append("TargetReached")
    if sw & SW_SETPOINT_ACK:
        flags.append("SetpointAck")
    if sw & SW_INTERNAL_LIMIT:
        flags.append("IntLimit")

    flag_str = ", ".join(flags) if flags else "-"
    print(f"  SW=0x{sw:04X}  State: {state}  [{flag_str}]")


def wait_for_state(expected_mask, expected_val, timeout_ms=2000):
    """Wait until statusword masked value matches. Returns final SW or None."""
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        result = recv_tpdo1(timeout_ms=int((deadline - time.time()) * 1000))
        if result is None:
            print("  [TIMEOUT] No TPDO received")
            return None
        sw, vel = result
        if (sw & expected_mask) == expected_val:
            return sw
    print(f"  [TIMEOUT] State not reached within {timeout_ms}ms")
    return None


# ===========================================================================
# Test Functions
# ===========================================================================
def test_read_status():
    """Menu 1: Read Statusword via SDO and TPDO."""
    print("\n--- Read Statusword ---")

    # SDO read
    sw = sdo_read(0x6041)
    if sw is not None:
        print("  [SDO] ", end="")
        print_statusword(sw)
    else:
        print("  [SDO] Failed to read statusword")

    # Try to get one TPDO
    result = recv_tpdo1(timeout_ms=2000)
    if result:
        sw, vel = result
        print("  [TPDO]", end=" ")
        print_statusword(sw)
        print(f"  Velocity Actual = {vel} ({vel / 1000.0:.1f} rpm)")
    else:
        print("  [TPDO] No TPDO received (may need SYNC or event trigger)")


def test_state_machine():
    """Menu 2: Automatic state machine sequence to OPERATION_ENABLED."""
    print("\n--- State Machine Sequence ---")

    # Step 1: Read current state
    print("\n[Step 0] Current state:")
    sw = sdo_read(0x6041)
    if sw is not None:
        print_statusword(sw)
    state = decode_state(sw) if sw else "Unknown"

    # Step 2: Shutdown -> READY_TO_SWITCH_ON
    if state == "SWITCH_ON_DISABLED":
        print("\n[Step 1] Sending Shutdown (CW=0x06)...")
        send_rpdo1(CW_SHUTDOWN)
        sw = wait_for_state(0x006F, 0x0021, 2000)
        if sw is not None:
            print("  -> ", end="")
            print_statusword(sw)
        else:
            print("  FAILED to reach READY_TO_SWITCH_ON")
            return

    # Step 3: Switch On -> SWITCHED_ON
    print("\n[Step 2] Sending Switch On (CW=0x07)...")
    send_rpdo1(CW_SWITCH_ON)
    sw = wait_for_state(0x006F, 0x0023, 2000)
    if sw is not None:
        print("  -> ", end="")
        print_statusword(sw)
    else:
        print("  FAILED to reach SWITCHED_ON")
        return

    # Step 4: Enable Operation -> OPERATION_ENABLED
    print("\n[Step 3] Sending Enable Operation (CW=0x0F)...")
    send_rpdo1(CW_ENABLE_OPERATION)
    sw = wait_for_state(0x006F, 0x0027, 3000)
    if sw is not None:
        print("  -> ", end="")
        print_statusword(sw)
        print("  Motor is now ENABLED!")
    else:
        print("  FAILED to reach OPERATION_ENABLED")
        # Try to read MC state via debug
        print("  Check UART output for MC_StartMotor1() result")


def test_set_velocity():
    """Menu 3: Set target velocity via RPDO."""
    print("\n--- Set Target Velocity ---")
    try:
        rpm = float(input("  Enter target velocity (rpm, 0=stop): "))
    except (ValueError, EOFError):
        print("  Invalid input")
        return

    target_raw = int(rpm * 1000)  # CIA402_VEL_SCALE = 1000
    cw = sdo_read(0x6040) or CW_ENABLE_OPERATION
    print(f"  Sending CW=0x{cw:04X}, TargetVelocity={target_raw} ({rpm:.0f} rpm)")
    send_rpdo1(cw, target_raw)

    time.sleep(0.5)
    result = recv_tpdo1(timeout_ms=2000)
    if result:
        sw, vel = result
        print_statusword(sw)
        print(f"  Velocity Actual = {vel} ({vel / 1000.0:.1f} rpm)")


def test_read_velocity():
    """Menu 4: Read velocity actual via SDO."""
    print("\n--- Read Velocity Actual ---")
    vel = sdo_read(0x606C)
    if vel is not None:
        if vel > 0x7FFFFFFF:
            vel -= 0x100000000
        print(f"  Velocity Actual = {vel} ({vel / 1000.0:.1f} rpm)")
    else:
        print("  Failed to read velocity")


def test_change_mode():
    """Menu 5: Change operation mode via SDO."""
    print("\n--- Change Operation Mode ---")
    print("  Available modes:")
    for code, name in MODE_NAMES.items():
        print(f"    {code}: {name}")
    try:
        mode = int(input("  Enter mode number: "))
    except (ValueError, EOFError):
        print("  Invalid input")
        return

    if mode not in MODE_NAMES:
        print(f"  Unknown mode: {mode}")
        return

    ok = sdo_write(0x6060, 0, mode, data_size=1)
    if ok:
        print(f"  Written mode = {mode} ({MODE_NAMES[mode]})")
        time.sleep(0.1)
        display = sdo_read(0x6061)
        if display is not None:
            display_name = MODE_NAMES.get(display, f"Unknown({display})")
            print(f"  Mode Display = {display} ({display_name})")
        else:
            print("  Could not read mode display")
    else:
        print("  SDO write failed")


def test_read_position():
    """Menu 6: Read position actual via SDO."""
    print("\n--- Read Position Actual ---")
    pos = sdo_read(0x6064)
    if pos is not None:
        if pos > 0x7FFFFFFF:
            pos -= 0x100000000
        print(f"  Position Actual = {pos} ({pos / 1000.0:.3f} rad)")
    else:
        print("  Failed to read position")


def test_quick_stop():
    """Menu 7: Quick Stop test."""
    print("\n--- Quick Stop Test ---")

    # First make sure we are in OPERATION_ENABLED
    sw = sdo_read(0x6041)
    if sw is None:
        print("  Cannot read statusword")
        return

    state = decode_state(sw)
    if state != "OPERATION_ENABLED":
        print(f"  Current state: {state}")
        print("  Please enable operation first (menu 2)")
        return

    print("  Sending Quick Stop (CW=0x02)...")
    send_rpdo1(CW_QUICK_STOP)
    sw = wait_for_state(0x006F, 0x0007, 3000)
    if sw is not None:
        print("  -> QUICK_STOP_ACTIVE")
        print_statusword(sw)

        # Wait for motor to stop
        print("  Waiting for motor to stop...")
        sw = wait_for_state(0x006F, 0x0040, 5000)
        if sw is not None:
            print("  -> SWITCH_ON_DISABLED")
            print_statusword(sw)
        else:
            print("  Motor did not reach SWITCH_ON_DISABLED")
    else:
        print("  FAILED to enter QUICK_STOP_ACTIVE")


def test_fault_reset():
    """Menu 8: Fault Reset test."""
    print("\n--- Fault Reset ---")
    sw = sdo_read(0x6041)
    if sw is None:
        print("  Cannot read statusword")
        return

    state = decode_state(sw)
    print(f"  Current state: {state}")
    print_statusword(sw)

    if state != "FAULT":
        print("  Not in FAULT state. No reset needed.")
        print("  To test: trigger a fault condition, then use this function.")
        return

    print("  Sending Fault Reset (CW=0x80, then CW=0x00)...")
    # Send fault reset (bit7=1)
    send_rpdo1(CW_FAULT_RESET)
    time.sleep(0.1)
    # Clear fault reset bit
    send_rpdo1(0x0000)
    time.sleep(0.2)

    sw = wait_for_state(0x006F, 0x0040, 2000)
    if sw is not None:
        print("  -> SWITCH_ON_DISABLED (fault reset OK)")
        print_statusword(sw)
    else:
        new_sw = sdo_read(0x6041)
        if new_sw:
            print(f"  Current state: {decode_state(new_sw)}")
            print_statusword(new_sw)
        else:
            print("  Fault reset may have failed")


def test_monitor():
    """Menu 9: Continuous monitor."""
    print("\n--- Continuous Monitor (Ctrl+C to stop) ---")
    print("  Time       | SW     | State                  | Velocity (rpm)")
    print("  -----------+--------+------------------------+---------------")

    start = time.time()
    try:
        while True:
            result = recv_tpdo1(timeout_ms=2000)
            if result:
                sw, vel = result
                elapsed = time.time() - start
                state = decode_state(sw)
                rpm = vel / 1000.0
                print(f"  {elapsed:8.2f}s  | 0x{sw:04X} | {state:22s} | {rpm:10.1f}")
            else:
                elapsed = time.time() - start
                print(f"  {elapsed:8.2f}s  | ---- no TPDO ----")
    except KeyboardInterrupt:
        print("\n  Monitor stopped.")


def test_disable():
    """Menu 10: Disable motor (return to SWITCH_ON_DISABLED)."""
    print("\n--- Disable Motor ---")
    sw = sdo_read(0x6041)
    state = decode_state(sw) if sw else "Unknown"
    print(f"  Current state: {state}")

    if state == "OPERATION_ENABLED":
        print("  Sending Disable Voltage (CW=0x00)...")
        send_rpdo1(CW_DISABLE_VOLTAGE)
        time.sleep(0.2)
    elif state == "SWITCHED_ON":
        print("  Sending Disable Voltage (CW=0x00)...")
        send_rpdo1(CW_DISABLE_VOLTAGE)
        time.sleep(0.2)
    elif state == "READY_TO_SWITCH_ON":
        print("  Sending Disable Voltage (CW=0x00)...")
        send_rpdo1(CW_DISABLE_VOLTAGE)
        time.sleep(0.2)

    sw = sdo_read(0x6041)
    if sw:
        print_statusword(sw)


def test_raw_sdo():
    """Menu 11: Raw SDO read/write."""
    print("\n--- Raw SDO Access ---")
    print("  1: Read")
    print("  2: Write")
    try:
        choice = input("  Choose: ").strip()
        index = int(input("  Index (hex, e.g. 6041): "), 16)
        sub = int(input("  Subindex (dec): "))
    except (ValueError, EOFError):
        print("  Invalid input")
        return

    if choice == "1":
        val = sdo_read(index, sub)
        if val is not None:
            print(f"  0x{index:04X}:{sub} = {val} (0x{val & 0xFFFFFFFF:08X})")
    elif choice == "2":
        try:
            val = int(input("  Value (dec or 0x hex): "), 0)
            size = int(input("  Size (1/2/4 bytes): "))
        except (ValueError, EOFError):
            print("  Invalid input")
            return
        ok = sdo_write(index, sub, val, data_size=size)
        print(f"  Write {'OK' if ok else 'FAILED'}")
    else:
        print("  Invalid choice")


# ===========================================================================
# Main Menu
# ===========================================================================
def main():
    print("=" * 60)
    print("  CiA 402 CANopen Test Tool")
    print(f"  Node ID={NODE_ID} (0x{NODE_ID:02X}), 500kbps, PCAN")
    print(f"  RPDO1=0x{RPDO1_ID:03X}  TPDO1=0x{TPDO1_ID:03X}")
    print(f"  SDO TX=0x{SDO_TX_ID:03X}  SDO RX=0x{SDO_RX_ID:03X}")
    print("=" * 60)

    can_init()

    # Flush stale messages
    print("Flushing CAN buffer...")
    while can_recv(timeout_ms=200):
        pass
    print("Ready.\n")

    while True:
        print("\n" + "-" * 50)
        print("  1. Read Statusword (SDO + TPDO)")
        print("  2. State Machine Enable (auto SOD->OE)")
        print("  3. Set Target Velocity (0x60FF)")
        print("  4. Read Velocity Actual (0x606C)")
        print("  5. Change Operation Mode (0x6060)")
        print("  6. Read Position Actual (0x6064)")
        print("  7. Quick Stop Test")
        print("  8. Fault Reset")
        print("  9. Continuous Monitor")
        print(" 10. Disable Motor")
        print(" 11. Raw SDO Read/Write")
        print("  0. Exit")
        print("-" * 50)

        try:
            choice = input("  Choice: ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "1":
            test_read_status()
        elif choice == "2":
            test_state_machine()
        elif choice == "3":
            test_set_velocity()
        elif choice == "4":
            test_read_velocity()
        elif choice == "5":
            test_change_mode()
        elif choice == "6":
            test_read_position()
        elif choice == "7":
            test_quick_stop()
        elif choice == "8":
            test_fault_reset()
        elif choice == "9":
            test_monitor()
        elif choice == "10":
            test_disable()
        elif choice == "11":
            test_raw_sdo()
        elif choice == "0":
            break
        else:
            print("  Invalid choice")

    print("\nShutting down...")
    if bus:
        bus.shutdown()
    print("Bye.")


if __name__ == "__main__":
    main()
