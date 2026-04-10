#!/usr/bin/env python3
import serial
import time
import sys
import tty
import termios

PORT = "/dev/ttyACM0"
BAUDRATE = 9600

# Initialize angles safely
angles = [90, 90, 90, 15] 
active_servo = 0
servo_names = ["0: BASE    ", "1: SHOULDER", "2: ELBOW   ", "3: GRIPPER "]
ser = None

def connect_serial():
    global ser
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=1)
            time.sleep(2) # Wait for Arduino to boot
            return True
        except Exception:
            time.sleep(1)
            print(".", end="", flush=True)

print(f"Connecting to {PORT}...", end="")
connect_serial()
print("\n[+] Connected!")

print("\n" + "="*40)
print("     ARMORED SERVO CALIBRATOR")
print("="*40)
print("Controls:")
print(" [0, 1, 2, 3] : Select active servo")
print(" [w] / [s]    : Nudge angle UP/DOWN by 1 degree")
print(" [a] / [d]    : Jump angle DOWN/UP by 5 degrees")
print(" [q]          : Quit calibrator")
print("="*40)

def send_command(idx, ang):
    global ser
    cmd = f"{idx},{ang}\n"
    ser.write(cmd.encode('ascii'))

def print_status():
    status = f"\rActive: {servo_names[active_servo]} | Current Angle: [{angles[active_servo]:03d}°] "
    sys.stdout.write(status)
    sys.stdout.flush()

def getch():
    """Reads a single keypress instantly."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Send initial safe angles
try:
    for i in range(4):
        send_command(i, angles[i])
        time.sleep(0.1)
except OSError:
    pass # Ignore initial startup bumps

print_status()

try:
    while True:
        char = getch().lower()
        
        if char == 'q':
            print("\nExiting calibrator.")
            break
            
        elif char in ['0', '1', '2', '3']:
            active_servo = int(char)
            
        elif char == 'w': # +1 degree
            angles[active_servo] = min(180, angles[active_servo] + 1)
            
        elif char == 's': # -1 degree
            angles[active_servo] = max(0, angles[active_servo] - 1)
            
        elif char == 'd': # +5 degrees
            angles[active_servo] = min(180, angles[active_servo] + 5)
            
        elif char == 'a': # -5 degrees
            angles[active_servo] = max(0, angles[active_servo] - 5)

        # Attempt to send the command. If the Arduino browns out, catch the error!
        if char in ['w', 's', 'a', 'd']:
            try:
                send_command(active_servo, angles[active_servo])
                print_status()
            except OSError:
                print("\n\n[!] STALL DETECTED! Servo pulled too much current.")
                print(f"[!] The absolute limit for {servo_names[active_servo].strip()} is around {angles[active_servo]}°.")
                print("[!] Waiting for Arduino to reboot...", end="")
                
                # Back the angle off by 3 degrees automatically so it doesn't immediately stall again
                if char in ['w', 'd']: 
                    angles[active_servo] = max(0, angles[active_servo] - 3)
                else: 
                    angles[active_servo] = min(180, angles[active_servo] + 3)
                
                if ser: ser.close()
                connect_serial()
                print(f"\n[+] Reconnected! Backed off to safe angle: {angles[active_servo]}°")
                
                # Re-apply the safe angle
                send_command(active_servo, angles[active_servo])
                print_status()

except KeyboardInterrupt:
    print("\nExiting calibrator.")
finally:
    if ser: ser.close()