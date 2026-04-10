#!/usr/bin/env python3
"""
Automated CiA 402 test - non-interactive, runs full test sequence.
Sends results to stdout, designed to be run by Claude Code.
"""

import struct
import time
import sys
import can
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

NODE_ID = 0x10
RPDO1_ID = 0x200 + NODE_ID  # 0x210
TPDO1_ID = 0x180 + NODE_ID  # 0x190
SDO_TX_ID = 0x600 + NODE_ID # 0x610
SDO_RX_ID = 0x580 + NODE_ID # 0x590

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"

bus = None
results = []


def log(test_name, status, detail=""):
    results.append((test_name, status, detail))
    icon = {"PASS": "+", "FAIL": "x", "SKIP": "-"}[status]
    print(f"  [{icon}] {test_name}: {detail}")


def can_init():
    global bus
    bus = can.interface.Bus(interface="pcan", bitrate=500000, channel="PCAN_USBBUS1")
    # Flush
    while bus.recv(timeout=0.1):
        pass


def can_recv_match(cob_id, timeout_ms):
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        remaining = max(0, deadline - time.time())
        msg = bus.recv(timeout=remaining)
        if msg and msg.arbitration_id == cob_id:
            return msg
    return None


def sdo_read(index, subindex=0, timeout_ms=2000):
    data = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
    bus.send(can.Message(arbitration_id=SDO_TX_ID, data=data, is_extended_id=False))
    msg = can_recv_match(SDO_RX_ID, timeout_ms)
    if not msg or len(msg.data) < 8:
        return None
    cmd = msg.data[0]
    rx_idx = msg.data[1] | (msg.data[2] << 8)
    rx_sub = msg.data[3]
    if rx_idx != index or rx_sub != subindex:
        return None
    if (cmd & 0xE0) == 0x80:
        abort = struct.unpack_from("<I", msg.data, 4)[0]
        return ("ABORT", abort)
    if (cmd & 0xE0) == 0x40:
        s = (cmd >> 4) & 1
        e = (cmd >> 3) & 1
        if s and e:
            n = (cmd >> 2) & 3
            valid = 4 - n
            raw = msg.data[4:8]
            if valid <= 1:
                return raw[0]
            elif valid <= 2:
                return struct.unpack_from("<H", raw)[0]
            else:
                return struct.unpack_from("<I", raw)[0]
    return None


def sdo_write(index, subindex, value, size=4, timeout_ms=2000):
    n = {4: 0, 3: 1, 2: 2, 1: 3}[size]
    cmd = 0x20 | (1 << 4) | (1 << 3) | (n << 2)
    payload = struct.pack("<I", value & (0xFFFFFFFF))
    data = bytes([cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]) + payload
    bus.send(can.Message(arbitration_id=SDO_TX_ID, data=data, is_extended_id=False))
    msg = can_recv_match(SDO_RX_ID, timeout_ms)
    if not msg or len(msg.data) < 4:
        return False
    rx_idx = msg.data[1] | (msg.data[2] << 8)
    rx_sub = msg.data[3]
    if rx_idx != index or rx_sub != subindex:
        return False
    if (msg.data[0] & 0xE0) == 0x80:
        return False
    return True


def send_rpdo1(cw, vel=0):
    data = struct.pack("<Hi", cw, vel)
    bus.send(can.Message(arbitration_id=RPDO1_ID, data=data, is_extended_id=False))


def recv_tpdo1(timeout_ms=2000):
    msg = can_recv_match(TPDO1_ID, timeout_ms)
    if msg and len(msg.data) >= 6:
        sw = struct.unpack_from("<H", msg.data, 0)[0]
        vel = struct.unpack_from("<i", msg.data, 2)[0]
        return (sw, vel)
    return None


def decode_state(sw):
    mask = sw & 0x006F
    states = {
        0x0000: "NOT_READY", 0x0040: "SOD", 0x0021: "RTSO",
        0x0023: "SO", 0x0027: "OE", 0x0007: "QSA",
        0x000F: "FRA", 0x0008: "FAULT"
    }
    return states.get(mask, f"?0x{mask:04X}")


def main():
    print("=" * 60)
    print("  CiA 402 Automated Test")
    print(f"  Node={NODE_ID}, 500kbps, RPDO=0x{RPDO1_ID:03X}, TPDO=0x{TPDO1_ID:03X}")
    print("=" * 60)

    # ---- Test 1: CAN init ----
    try:
        can_init()
        log("CAN Init", PASS, "PCAN opened at 500kbps")
    except Exception as e:
        log("CAN Init", FAIL, str(e))
        print("\nCannot proceed without CAN. Aborting.")
        sys.exit(1)

    # ---- Test 2: SDO read statusword ----
    print("\n--- Phase 1: SDO Communication ---")
    sw = sdo_read(0x6041)
    if sw is not None and not isinstance(sw, tuple):
        log("SDO Read Statusword", PASS, f"SW=0x{sw:04X} state={decode_state(sw)}")
    elif isinstance(sw, tuple) and sw[0] == "ABORT":
        log("SDO Read Statusword", FAIL, f"SDO abort code=0x{sw[1]:08X}")
    else:
        log("SDO Read Statusword", FAIL, "No SDO response - STM32 not responding")

    # ---- Test 3: SDO read device type ----
    dt = sdo_read(0x1000)
    if dt is not None and not isinstance(dt, tuple):
        log("SDO Read DeviceType", PASS, f"0x1000=0x{dt:08X}")
    else:
        log("SDO Read DeviceType", FAIL, "No response")

    # ---- Test 4: SDO read identity ----
    vid = sdo_read(0x1018, 1)
    if vid is not None and not isinstance(vid, tuple):
        log("SDO Read VendorID", PASS, f"0x1018:01=0x{vid:08X}")
    else:
        log("SDO Read VendorID", FAIL, "No response")

    # ---- Test 5: SDO read supported mode ----
    mode = sdo_read(0x6060)
    if mode is not None and not isinstance(mode, tuple):
        log("SDO Read OpMode", PASS, f"0x6060={mode}")
    else:
        log("SDO Read OpMode", FAIL, "No response")

    # If no SDO response at all, STM32 is not communicating
    if sw is None or isinstance(sw, tuple):
        print("\n!!! STM32 is not responding to SDO. Check:")
        print("    - STM32 is powered on")
        print("    - CAN bus wires (H/L) connected correctly")
        print("    - CAN transceiver on board")
        print("    - Firmware is flashed and running")
        log("Overall", FAIL, "STM32 not communicating")
        print_summary()
        return

    # ---- Test 6: TPDO receive ----
    print("\n--- Phase 2: PDO Communication ---")
    # Send a dummy RPDO to trigger potential TPDO response
    send_rpdo1(0x0000, 0)
    result = recv_tpdo1(timeout_ms=3000)
    if result:
        sw_tpdo, vel_tpdo = result
        log("TPDO1 Receive", PASS, f"SW=0x{sw_tpdo:04X} Vel={vel_tpdo}")
    else:
        log("TPDO1 Receive", FAIL, "No TPDO1 frame received within 3s")
        log("TPDO1 Note", SKIP, "TPDO may need event trigger; SDO works, so try SDO reads instead")

    # ---- Test 7: State Machine ----
    print("\n--- Phase 3: State Machine ---")
    current_sw = sdo_read(0x6041)
    if current_sw is None:
        log("State Machine", FAIL, "Lost communication")
        print_summary()
        return

    current_state = decode_state(current_sw)
    print(f"  Current state: {current_state} (SW=0x{current_sw:04X})")

    # Transition to OPERATION_ENABLED step by step
    # Step A: If in SOD, send Shutdown
    if current_state == "SOD":
        print("  Sending Shutdown (CW=0x06)...")
        send_rpdo1(0x0006)
        time.sleep(0.3)
        sw_check = sdo_read(0x6041)
        if sw_check is not None:
            new_state = decode_state(sw_check)
            log("SOD -> RTSO", PASS if new_state == "RTSO" else FAIL,
                f"SW=0x{sw_check:04X} state={new_state}")
            current_state = new_state
        else:
            log("SOD -> RTSO", FAIL, "Lost communication after shutdown")
            print_summary()
            return
    elif current_state == "RTSO":
        log("SOD -> RTSO", SKIP, "Already in RTSO")
    elif current_state == "SO":
        log("SOD -> RTSO", SKIP, "Already in SO")
    elif current_state == "OE":
        log("State Machine", SKIP, "Already in OPERATION_ENABLED")
        current_state = "OE"
    elif current_state == "FAULT":
        log("State Machine", SKIP, "In FAULT state, attempting reset...")
        send_rpdo1(0x0080)
        time.sleep(0.1)
        send_rpdo1(0x0000)
        time.sleep(0.3)
        sw_check = sdo_read(0x6041)
        if sw_check:
            log("Fault Reset", PASS if decode_state(sw_check) == "SOD" else FAIL,
                f"SW=0x{sw_check:04X}")
            current_state = decode_state(sw_check)
        else:
            log("Fault Reset", FAIL, "No response")
            print_summary()
            return
    else:
        log("State Machine", FAIL, f"Unexpected state: {current_state}")
        print_summary()
        return

    # Step B: RTSO -> SO
    if current_state == "RTSO":
        print("  Sending Switch On (CW=0x07)...")
        send_rpdo1(0x0007)
        time.sleep(0.3)
        sw_check = sdo_read(0x6041)
        if sw_check is not None:
            new_state = decode_state(sw_check)
            log("RTSO -> SO", PASS if new_state == "SO" else FAIL,
                f"SW=0x{sw_check:04X} state={new_state}")
            current_state = new_state
        else:
            log("RTSO -> SO", FAIL, "Lost communication")
            print_summary()
            return

    # Step C: SO -> OE
    if current_state == "SO":
        print("  Sending Enable Operation (CW=0x0F)...")
        send_rpdo1(0x000F)
        time.sleep(1.0)  # Motor start takes time
        sw_check = sdo_read(0x6041)
        if sw_check is not None:
            new_state = decode_state(sw_check)
            if new_state == "OE":
                log("SO -> OE", PASS, f"SW=0x{sw_check:04X} MOTOR ENABLED")
            else:
                log("SO -> OE", FAIL, f"SW=0x{sw_check:04X} state={new_state}")
                # Read MC faults for debug
                faults = sdo_read(0x1001)
                print(f"  Error register: 0x{faults:02X}" if faults else "  Cannot read error register")
                print_summary()
                return
            current_state = new_state
        else:
            log("SO -> OE", FAIL, "Lost communication after enable")
            print_summary()
            return

    # ---- Test 8: Velocity control ----
    if current_state == "OE":
        print("\n--- Phase 4: Velocity Control ---")

        # Read current velocity
        vel = sdo_read(0x606C)
        if vel is not None:
            rpm = vel / 1000.0
            log("Read Velocity", PASS, f"Current velocity = {vel} ({rpm:.1f} rpm)")
        else:
            log("Read Velocity", FAIL, "Cannot read velocity")

        # Set target velocity = 500 rpm
        target_rpm = 500
        target_raw = int(target_rpm * 1000)
        print(f"  Setting target velocity = {target_rpm} rpm (raw={target_raw})...")
        send_rpdo1(0x000F, target_raw)
        time.sleep(2.0)

        vel_after = sdo_read(0x606C)
        sw_after = sdo_read(0x6041)
        if vel_after is not None:
            rpm_after = vel_after / 1000.0
            running = abs(rpm_after) > 50
            log("Velocity Response", PASS if running else FAIL,
                f"Velocity = {vel_after} ({rpm_after:.1f} rpm)")
        else:
            log("Velocity Response", FAIL, "Cannot read velocity after command")

        if sw_after:
            log("State After Run", PASS if decode_state(sw_after) == "OE" else FAIL,
                f"SW=0x{sw_after:04X} state={decode_state(sw_after)}")

        # Stop motor
        print("  Stopping motor (target=0)...")
        send_rpdo1(0x000F, 0)
        time.sleep(2.0)

        vel_stop = sdo_read(0x606C)
        if vel_stop is not None:
            rpm_stop = vel_stop / 1000.0
            stopped = abs(rpm_stop) < 50
            log("Motor Stop", PASS if stopped else FAIL,
                f"Velocity = {vel_stop} ({rpm_stop:.1f} rpm)")
        else:
            log("Motor Stop", FAIL, "Cannot read velocity")

        # ---- Test 9: Disable ----
        print("\n--- Phase 5: Disable Motor ---")
        send_rpdo1(0x0000)  # Disable Voltage
        time.sleep(0.5)
        sw_final = sdo_read(0x6041)
        if sw_final:
            final_state = decode_state(sw_final)
            log("Disable Motor", PASS if final_state == "SOD" else FAIL,
                f"SW=0x{sw_final:04X} state={final_state}")
        else:
            log("Disable Motor", FAIL, "Lost communication")

    print_summary()
    bus.shutdown()


def print_summary():
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    passed = sum(1 for _, s, _ in results if s == PASS)
    failed = sum(1 for _, s, _ in results if s == FAIL)
    skipped = sum(1 for _, s, _ in results if s == SKIP)
    total = len(results)
    for name, status, detail in results:
        icon = {"PASS": "+", "FAIL": "!", "SKIP": "-"}[status]
        print(f"  [{icon}] {name}: {detail}")
    print("-" * 60)
    print(f"  Total: {total}  PASS: {passed}  FAIL: {failed}  SKIP: {skipped}")
    if failed == 0:
        print("  ALL TESTS PASSED!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
