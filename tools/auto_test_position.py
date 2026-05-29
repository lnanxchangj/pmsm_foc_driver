#!/usr/bin/env python3
"""
Automated CiA 402 Position Mode Test (PP mode).
Sequence: Set PP mode -> Enable motor -> Move 90 degrees -> Wait target reached -> Disable motor.
"""

import struct
import time
import sys
import math
import can
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

NODE_ID = 0x01
RPDO1_ID = 0x200 + NODE_ID  # 0x210
TPDO1_ID = 0x180 + NODE_ID  # 0x190
SDO_TX_ID = 0x600 + NODE_ID # 0x610
SDO_RX_ID = 0x580 + NODE_ID # 0x590

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"

# 90 degrees in radians: pi/2 ≈ 1.5708 rad, raw = 1571 (CIA402_POS_SCALE=1000)
TARGET_ANGLE_DEG = 90.0
TARGET_ANGLE_RAD = math.radians(TARGET_ANGLE_DEG)
TARGET_POS_RAW = int(TARGET_ANGLE_RAD * 1000)  # 1571

bus = None
results = []


def log(test_name, status, detail=""):
    results.append((test_name, status, detail))
    icon = {"PASS": "+", "FAIL": "x", "SKIP": "-"}[status]
    print(f"  [{icon}] {test_name}: {detail}")


def can_init():
    global bus
    bus = can.interface.Bus(interface="pcan", bitrate=500000, channel="PCAN_USBBUS1")
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
        s = cmd & 1
        e = (cmd >> 1) & 1
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


def decode_state(sw):
    mask = sw & 0x006F
    states = {
        0x0000: "NOT_READY", 0x0060: "SOD", 0x0021: "RTSO",
        0x0023: "SO", 0x0027: "OE", 0x0007: "QSA",
        0x002F: "FRA", 0x0008: "FAULT"
    }
    return states.get(mask, f"?0x{mask:04X}")


def wait_for_state(target_state, timeout_ms=3000):
    """Poll SDO statusword until decode_state matches target_state."""
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        sw = sdo_read(0x6041)
        if sw is not None and not isinstance(sw, tuple):
            if decode_state(sw) == target_state:
                return sw
        time.sleep(0.05)
    return None


def main():
    print("=" * 60)
    print("  CiA 402 Position Mode (PP) Automated Test")
    print(f"  Node={NODE_ID}, 500kbps, Target={TARGET_ANGLE_DEG} deg ({TARGET_POS_RAW} raw)")
    print("=" * 60)

    # ---- Phase 1: CAN Init ----
    print("\n--- Phase 1: CAN Init ---")
    try:
        can_init()
        log("CAN Init", PASS, "PCAN opened at 500kbps")
    except Exception as e:
        log("CAN Init", FAIL, str(e))
        print("\nCannot proceed without CAN. Aborting.")
        sys.exit(1)

    # ---- Phase 2: SDO Communication Check ----
    print("\n--- Phase 2: SDO Communication ---")
    sw = sdo_read(0x6041)
    if sw is not None and not isinstance(sw, tuple):
        log("SDO Read Statusword", PASS, f"SW=0x{sw:04X} state={decode_state(sw)}")
    elif isinstance(sw, tuple) and sw[0] == "ABORT":
        log("SDO Read Statusword", FAIL, f"SDO abort code=0x{sw[1]:08X}")
        print_summary()
        return
    else:
        log("SDO Read Statusword", FAIL, "No SDO response - STM32 not responding")
        print_summary()
        return

    # Read current position
    pos = sdo_read(0x6064)
    if pos is not None and not isinstance(pos, tuple):
        pos_rad = pos / 1000.0
        pos_deg = math.degrees(pos_rad)
        log("Read Initial Position", PASS, f"pos={pos} ({pos_rad:.4f} rad / {pos_deg:.1f} deg)")
    else:
        log("Read Initial Position", FAIL, "Cannot read position")

    # Read current mode
    mode = sdo_read(0x6060)
    if mode is not None:
        mode_names = {0: "NoMode", 1: "PP", 3: "PV", 4: "PT", 6: "HM", 8: "CSP", 9: "CSV", 10: "CST"}
        log("Read Current Mode", PASS, f"mode={mode} ({mode_names.get(mode, '?')})")
    else:
        log("Read Current Mode", FAIL, "No response")

    # ---- Phase 3: Set PP Mode ----
    print("\n--- Phase 3: Set PP Mode ---")
    ok = sdo_write(0x6060, 0, 1, size=1)  # PP mode = 1
    if ok:
        time.sleep(0.1)
        mode_disp = sdo_read(0x6061)
        if mode_disp is not None and mode_disp == 1:
            log("Set PP Mode", PASS, f"0x6061={mode_disp} (PP mode confirmed)")
        else:
            log("Set PP Mode", FAIL, f"0x6061={mode_disp} (mode display mismatch)")
    else:
        log("Set PP Mode", FAIL, "SDO write 0x6060 failed")
        print_summary()
        return

    # ---- Phase 4: State Machine Enable ----
    print("\n--- Phase 4: State Machine Enable ---")
    current_sw = sdo_read(0x6041)
    if current_sw is None:
        log("State Machine", FAIL, "Lost communication")
        print_summary()
        return

    current_state = decode_state(current_sw)
    print(f"  Current state: {current_state} (SW=0x{current_sw:04X})")

    # Handle FAULT state
    if current_state == "FAULT":
        print("  In FAULT, attempting reset...")
        send_rpdo1(0x0080)
        time.sleep(0.1)
        send_rpdo1(0x0000)
        time.sleep(0.3)
        current_sw = sdo_read(0x6041)
        if current_sw:
            current_state = decode_state(current_sw)
            log("Fault Reset", PASS if current_state == "SOD" else FAIL,
                f"SW=0x{current_sw:04X} state={current_state}")
        else:
            log("Fault Reset", FAIL, "No response")
            print_summary()
            return

    # SOD -> RTSO
    if current_state == "SOD":
        print("  Sending Shutdown (CW=0x06)...")
        send_rpdo1(0x0006)
        sw_check = wait_for_state("RTSO", 2000)
        if sw_check:
            log("SOD -> RTSO", PASS, f"SW=0x{sw_check:04X}")
            current_state = "RTSO"
        else:
            log("SOD -> RTSO", FAIL, "Timeout")
            print_summary()
            return
    elif current_state in ("RTSO", "SO", "OE"):
        log("SOD -> RTSO", SKIP, f"Already in {current_state}")

    # RTSO -> SO
    if current_state == "RTSO":
        print("  Sending Switch On (CW=0x07)...")
        send_rpdo1(0x0007)
        sw_check = wait_for_state("SO", 2000)
        if sw_check:
            log("RTSO -> SO", PASS, f"SW=0x{sw_check:04X}")
            current_state = "SO"
        else:
            log("RTSO -> SO", FAIL, "Timeout")
            print_summary()
            return

    # SO -> OE
    if current_state == "SO":
        print("  Sending Enable Operation (CW=0x0F)...")
        send_rpdo1(0x000F)
        time.sleep(1.0)
        sw_check = sdo_read(0x6041)
        if sw_check is not None:
            new_state = decode_state(sw_check)
            if new_state == "OE":
                log("SO -> OE", PASS, f"SW=0x{sw_check:04X} MOTOR ENABLED")
                current_state = "OE"
            else:
                log("SO -> OE", FAIL, f"SW=0x{sw_check:04X} state={new_state}")
                faults = sdo_read(0x1001)
                print(f"  Error register: 0x{faults:02X}" if faults else "  Cannot read error register")
                print_summary()
                return
        else:
            log("SO -> OE", FAIL, "Lost communication")
            print_summary()
            return
    elif current_state == "OE":
        log("SO -> OE", SKIP, "Already OE")

    # ---- Phase 5: Position Control (Move 90 degrees) ----
    if current_state == "OE":
        print(f"\n--- Phase 5: Position Control ({TARGET_ANGLE_DEG} deg) ---")

        # Read position before move
        pos_before = sdo_read(0x6064)
        if pos_before is not None and not isinstance(pos_before, tuple):
            pos_before_rad = pos_before / 1000.0
            print(f"  Position before move: {pos_before} ({pos_before_rad:.4f} rad)")
        else:
            pos_before = 0

        # Write target position via SDO: 0x607A = 1571 (90 deg)
        print(f"  Writing target position: 0x607A = {TARGET_POS_RAW} ({TARGET_ANGLE_DEG} deg)...")
        ok = sdo_write(0x607A, 0, TARGET_POS_RAW, size=4)
        if ok:
            log("Write Target Position", PASS, f"0x607A={TARGET_POS_RAW}")
        else:
            log("Write Target Position", FAIL, "SDO write failed")
            # Try to disable before exit
            send_rpdo1(0x0000)
            print_summary()
            return

        # Monitor position until target reached (SW bit10)
        print("  Waiting for target reached (SW bit10)...")
        t_start = time.time()
        reached = False
        monitor_log = []

        for i in range(100):  # max 10 seconds
            time.sleep(0.1)
            sw = sdo_read(0x6041)
            pos = sdo_read(0x6064)

            if sw is None:
                continue

            elapsed = time.time() - t_start
            target_reached = (sw & (1 << 10)) != 0
            setpoint_ack = (sw & (1 << 12)) != 0
            state = decode_state(sw)

            if pos is not None and not isinstance(pos, tuple):
                pos_rad = pos / 1000.0
                pos_deg = math.degrees(pos_rad)
                error_deg = TARGET_ANGLE_DEG - pos_deg
            else:
                pos_rad = 0
                pos_deg = 0
                error_deg = 0

            if i % 5 == 0:  # Print every 0.5s
                print(f"    {elapsed:5.1f}s  SW=0x{sw:04X} state={state:3s} "
                      f"pos={pos_deg:7.1f} deg  err={error_deg:+7.1f} deg  "
                      f"TR={'Y' if target_reached else 'N'} ACK={'Y' if setpoint_ack else 'N'}")

            monitor_log.append((elapsed, pos_deg, error_deg, state))

            if target_reached:
                reached = True
                print(f"    >>> TARGET REACHED at {elapsed:.1f}s, pos={pos_deg:.1f} deg")
                break

            # Check if we left OE state (fault etc)
            if state not in ("OE",):
                print(f"    !!! Unexpected state: {state}")
                break

        if reached:
            final_pos = sdo_read(0x6064)
            if final_pos is not None and not isinstance(final_pos, tuple):
                final_deg = math.degrees(final_pos / 1000.0)
                error = abs(final_deg - TARGET_ANGLE_DEG)
                log("Position Target Reached", PASS,
                    f"final={final_deg:.1f} deg, error={error:.2f} deg, time={time.time()-t_start:.1f}s")
            else:
                log("Position Target Reached", PASS, "SW bit10 set but cannot read final position")
        else:
            final_pos = sdo_read(0x6064)
            if final_pos is not None and not isinstance(final_pos, tuple):
                final_deg = math.degrees(final_pos / 1000.0)
            else:
                final_deg = 0
            log("Position Target Reached", FAIL,
                f"Timeout after {time.time()-t_start:.1f}s, pos={final_deg:.1f} deg")

        # Wait a moment for stability
        time.sleep(1.0)

        # ---- Phase 6: Disable Motor ----
        print("\n--- Phase 6: Disable Motor ---")
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
