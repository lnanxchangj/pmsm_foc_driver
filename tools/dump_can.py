import can
import time
import sys

def main():
    try:
        bus = can.interface.Bus(interface="pcan", bitrate=500000, channel="PCAN_USBBUS1")
        print("Listening on PCAN_USBBUS1 at 500kbps for 15 seconds...")
        print("PLEASE RESET THE STM32 BOARD NOW!")
        sys.stdout.flush()
        
        deadline = time.time() + 15.0
        while time.time() < deadline:
            msg = bus.recv(timeout=1.0)
            if msg:
                print(f"Received: {msg}")
                sys.stdout.flush()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
