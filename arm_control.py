#!/usr/bin/env python3
import serial
import time
import sys

# Setup Serial connection (Ensure this matches the Mega's port)
PORT = '/dev/ttyACM0' 
BAUDRATE = 115200 # Must match the Arduino code's Serial.begin(115200)

try:
    print(f"Connecting to Arm on {PORT} at {BAUDRATE} baud...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2) # Give Arduino time to reboot after connection
    print("Connection established!\n")
except Exception as e:
    print(f"Failed to connect: {e}")
    sys.exit(1)

def send_command(command_str):
    """Encodes and sends the command, then prints the Arduino's response."""
    full_cmd = f"{command_str}\n"
    ser.write(full_cmd.encode('ascii'))
    
    # Give the arm a moment to process and return text
    time.sleep(0.1) 
    while ser.in_waiting > 0:
        response = ser.readline().decode('ascii', errors='replace').strip()
        print(f"Arduino: {response}")

def print_menu():
    print("-" * 40)
    print("ROBOTIC ARM CONTROLLER")
    print("-" * 40)
    print("Format: [Joint] [Angle]")
    print("  B <angle> : Move Base (0-180)")
    print("  S <angle> : Move Shoulder (0-180)")
    print("  E <angle> : Move Elbow (0-180)")
    print("  G <angle> : Move Gripper (0-30 max)")
    print("  V <ms>    : Set Speed/Delay per degree")
    print("  H         : Home all servos")
    print("  Q         : Quit")
    print("-" * 40)

if __name__ == '__main__':
    print_menu()
    
    try:
        while True:
            user_input = input("Enter command: ").strip().upper()
            
            if user_input == 'Q':
                print("Exiting...")
                break
                
            if user_input == 'H':
                send_command("H")
                continue
                
            # Parse inputs like "B 45" or "B45"
            if len(user_input) >= 2:
                joint = user_input[0]
                # Extract the number part and strip any spaces
                val_str = user_input[1:].strip() 
                
                if joint in ['B', 'S', 'E', 'G', 'V'] and val_str.isdigit():
                    val = int(val_str)
                    
                    # The Arduino code specifically requires EXACTLY 3 digits
                    # f"{val:03d}" turns 45 into "045"
                    formatted_command = f"{joint}{val:03d}"
                    print(f"Sending: {formatted_command}")
                    send_command(formatted_command)
                else:
                    print("Invalid format. Use Letter + Number (e.g., B 90)")
            else:
                print("Input too short.")
                
    except KeyboardInterrupt:
        print("\nExiting via KeyboardInterrupt.")
    finally:
        if ser.is_open:
            ser.close()