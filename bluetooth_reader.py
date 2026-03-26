import serial
import time
import sys

# --- Configuration ---
# Configure serial port for Bluetooth module.
# Windows: 'COM3', 'COM4', etc.
# macOS: '/dev/tty.Bluetooth-Incoming-Port' or '/dev/tty.HC-05-SPPDev'
# Linux: Usually '/dev/rfcomm0' (after pairing and binding)
#
# How to find the port:
# 1. Pair your Bluetooth module with your computer.
# 2. On Windows: Go to Device Manager -> Ports (COM & LPT). Look for a "Standard Serial over Bluetooth link" or similar.
# 3. On macOS: System Preferences -> Bluetooth -> Advanced. Look for the serial port associated with your device.
#    You might also check /dev/tty.* after connecting.
# 4. On Linux: After pairing, you might need to create an rfcomm bind:
#    `sudo rfcomm bind /dev/rfcomm0 <bluetooth_mac_address>`
#    You can find the MAC address using `hcitool scan` or `bluetoothctl`.
#    Then the port will be '/dev/rfcomm0'.
#
BLUETOOTH_PORT = 'COMx'  # <--- REPLACE THIS WITH YOUR ACTUAL BLUETOOTH SERIAL PORT
BAUD_RATE = 9600         # Must match BT_BAUD_RATE in config_pins.h

# --- Main Script ---
def read_bluetooth_data():
    try:
        # Open the serial port
        ser = serial.Serial(BLUETOOTH_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Bluetooth port: {BLUETOOTH_PORT} at {BAUD_RATE} baud.")
        print("Waiting for data from Arduino...")

        while True:
            if ser.in_waiting > 0:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"Received: {line}")
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"Error: Could not open or communicate with serial port {BLUETOOTH_PORT}.")
        print(f"Please check if the port is correct and if the device is connected/paired.")
        print(f"Details: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    read_bluetooth_data()