"""
Serial Logger for Arduino Step Response Data
---------------------------------------------
Reads serial output from the Arduino and saves it to a text file.
Automatically stops when it detects "Printing is done" from the Arduino.

Usage:
  1. Update SERIAL_PORT below to match your Arduino port.
     (Find it in Arduino IDE -> Tools -> Port)
  2. Upload your sketch to the Arduino first.
  3. Close the Arduino Serial Monitor (only one program can use the port).
  4. Run:  python3 serial_logger.py
"""

import serial
import sys
import time
import glob

# ---- Configuration ----
SERIAL_PORT = None  # Set to your port, e.g. "/dev/cu.usbmodem14101", or leave None to auto-detect
BAUD_RATE = 115200
OUTPUT_FILE = "step_response_data.txt"
# ------------------------


def find_arduino_port():
    """Try to auto-detect the Arduino serial port on macOS."""
    ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
    if ports:
        return ports[0]
    return None


def main():
    port = SERIAL_PORT
    if port is None:
        port = find_arduino_port()
        if port is None:
            print("ERROR: Could not auto-detect Arduino port.")
            print("Available /dev/cu.* ports:")
            for p in glob.glob("/dev/cu.*"):
                print(f"  {p}")
            print("\nSet SERIAL_PORT in this script to your Arduino's port and try again.")
            sys.exit(1)

    print(f"Connecting to {port} at {BAUD_RATE} baud...")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        print("Make sure the Arduino Serial Monitor is closed and the port is correct.")
        sys.exit(1)

    time.sleep(2)  # Wait for Arduino to reset after serial connection
    print(f"Connected! Logging to '{OUTPUT_FILE}'...")
    print("Waiting for data (will auto-stop when 'Printing is done' is received)...\n")

    line_count = 0
    with open(OUTPUT_FILE, "w") as f:
        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                print(line)
                f.write(line + "\n")
                line_count += 1

                # Stop automatically after the Arduino finishes printing recorded data
                if "Printing is done" in line:
                    print(f"\n--- Recording complete! {line_count} lines saved to '{OUTPUT_FILE}' ---")
                    break

            except KeyboardInterrupt:
                print(f"\n--- Stopped by user. {line_count} lines saved to '{OUTPUT_FILE}' ---")
                break
            except Exception as e:
                print(f"Error reading serial: {e}")
                break

    ser.close()
    print("Serial port closed.")


if __name__ == "__main__":
    main()
