#!/usr/bin/env python3
"""
CG2111A Alex Robot - Integrated Pi Interface
Features: Auto-Reconnect Brownout Protection, Parameterized Arm Controls
"""

import struct
import serial
import time
import sys
import select
import alex_camera

from second_terminal import relay

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------
PORT     = "/dev/ttyACM0"
BAUDRATE = 9600
_ser = None

def openSerial():
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")

def closeSerial():
    global _ser
    if _ser and _ser.is_open:
        try:
            _ser.close()
        except:
            pass

# --- THE BROWNOUT BODYGUARD ---
def reconnectSerial():
    global _ser
    print("\n[!] HARDWARE DISCONNECT DETECTED (Brownout/Dead Battery).")
    print("[!] Attempting to reconnect to Arduino", end="")
    closeSerial()
    
    while True:
        try:
            time.sleep(1)
            _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
            print(f"\n[+] Arduino found at {PORT}! Waiting 2s for setup()...")
            time.sleep(2)
            print("[+] Connection restored. Ready to drive.\n")
            break
        except Exception:
            # Print a dot every second while waiting for Arduino to come back
            print(".", end="", flush=True)

# ----------------------------------------------------------------
# TPACKET CONSTANTS (Imported from shared file)
# ----------------------------------------------------------------
from packets import *

# ----------------------------------------------------------------
# FRAMING & COMMUNICATION
# ----------------------------------------------------------------
def computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result

def packFrame(packetType, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])

def unpackTPacket(raw):
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }

def receiveFrame():
    try:
        MAGIC_HI, MAGIC_LO = MAGIC[0], MAGIC[1]
        discarded = 0 
        while True:
            b = _ser.read(1)
            if not b: return None 
            discarded += 1
            if discarded > 200: return None 
            if b[0] != MAGIC_HI: continue
            b = _ser.read(1)
            if not b or b[0] != MAGIC_LO: continue

            raw = b''
            while len(raw) < TPACKET_SIZE:
                chunk = _ser.read(TPACKET_SIZE - len(raw))
                if not chunk: return None
                raw += chunk

            cs_byte = _ser.read(1)
            if not cs_byte or cs_byte[0] != computeChecksum(raw): continue
            return unpackTPacket(raw)
    except OSError:
        # If it crashes mid-read, return None and let the main loop trigger reconnect
        return None

def sendCommand(commandType, data=b'', params=None):
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    try:
        _ser.write(frame)
    except OSError:
        pass # Let the main loop catch the disconnect on the next pass

# ----------------------------------------------------------------
# E-STOP & STATE MANAGEMENT
# ----------------------------------------------------------------
_estop_state = STATE_RUNNING

def isEstopActive():
    return _estop_state == STATE_STOPPED

def printPacket(pkt):
    global _estop_state
    ptype, cmd = pkt['packetType'], pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            if pkt['params'][0] > 0: 
                print(f"Response: OK (Current Speed: {pkt['params'][0]})")
            else:
                print("Response: OK")
        elif cmd == RESP_STATUS:
            new_state = pkt['params'][0]
            if _estop_state == STATE_STOPPED and new_state == STATE_RUNNING:
                print(f"\n[{time.strftime('%H:%M:%S')}] LOG: E-Stop Cleared. Motors Enabled.")
            _estop_state = new_state
            print("Status: RUNNING" if new_state == STATE_RUNNING else "Status: STOPPED")
        elif cmd == RESP_COLOR:
            print(f"Color: R={pkt['params'][0]} Hz, G={pkt['params'][1]} Hz, B={pkt['params'][2]} Hz")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug: print(f"Arduino debug: {debug}")

# ----------------------------------------------------------------
# SENSOR HANDLERS
# ----------------------------------------------------------------
_camera = alex_camera.cameraOpen() 
_frames_remaining = 15

def handleColorCommand():
    if isEstopActive(): print("Refused: E-Stop Active."); return
    sendCommand(COMMAND_COLOR)

def handleCameraCommand():
    global _frames_remaining
    if isEstopActive(): print("Refused: E-Stop Active."); return
    if _frames_remaining <= 0: print("Refused: Frame limit reached."); return
    print("Capturing...")
    frame = alex_camera.captureGreyscaleFrame(_camera)
    alex_camera.renderGreyscaleFrame(frame)
    _frames_remaining -= 1

# ----------------------------------------------------------------
# MAIN INTERFACE
# ----------------------------------------------------------------
def handleUserInput(line):
    # Split the input. E.g., "g 5" becomes parts[0] = 'g', parts[1] = '5'
    parts = line.split()
    if not parts: return
    cmd = parts[0]

    if isEstopActive() and cmd != 'r':
        print("Refused: E-Stop is active. Press 'r' to reset.")
        return

    # --- Standard Commands ---
    if cmd == 'r': sendCommand(COMMAND_CLEAR_ESTOP)
    elif cmd == 'e': sendCommand(COMMAND_ESTOP)
    elif cmd == 'c': handleColorCommand()
    elif cmd == 'p': handleCameraCommand()
    elif cmd == 'w': sendCommand(COMMAND_FORWARD)
    elif cmd == 's' and len(parts) == 1: sendCommand(COMMAND_BACKWARD) # 's' alone is backward
    elif cmd == 'a': sendCommand(COMMAND_TURN_LEFT)
    elif cmd == 'd': sendCommand(COMMAND_TURN_RIGHT)
    elif cmd == '+': sendCommand(COMMAND_SPEED_UP)
    elif cmd == '-': sendCommand(COMMAND_SPEED_DOWN)
    elif cmd == 'h': sendCommand(COMMAND_STOP)

    # --- Arm Angle Commands (Format: letter angle) ---
    elif cmd == 'b' and len(parts) > 1: # Base (e.g., "b 90")
        sendCommand(COMMAND_SET_BASE, params=[int(parts[1])])
        
    elif cmd == 's' and len(parts) > 1: # Shoulder (e.g., "s 0")
        sendCommand(COMMAND_SET_SHOULDER, params=[int(parts[1])])
        
    elif cmd == 'l' and len(parts) > 1: # Elbow (e.g., "l 90")
        sendCommand(COMMAND_SET_ELBOW, params=[int(parts[1])])
        
    elif cmd == 'g' and len(parts) > 1: # Gripper (e.g., "g 5")
        sendCommand(COMMAND_SET_GRIPPER, params=[int(parts[1])])
        
    else: 
        print(f"Unknown command format: '{line}'")


def runCommandInterface():
    print("="*70)
    print("Controls: [w/s/a/d] Move  | [h] Stop      | [c/p] Sensors | [e/r] E-Stop")
    print("Arm:      [b <deg>] Base  | [s <deg>] Shld| [l <deg>] Elbw| [g <deg>] Grip")
    print("="*70)
    while True:
        try:
            relay.checkSecondTerminal(_ser)
            
            # The try block catches the Errno 5 exactly when in_waiting is called
            if _ser.in_waiting >= FRAME_SIZE:
                pkt = receiveFrame()
                if pkt: 
                    printPacket(pkt)
                    relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'], pkt['data'], pkt['params']))
                    
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline().strip().lower()
                if line: handleUserInput(line)
                
        except OSError:
            # THIS PREVENTS THE SCRIPT FROM CRASHING!
            reconnectSerial()
            
        time.sleep(0.05)

if __name__ == '__main__':
    openSerial()
    relay.start()
    
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if _camera: alex_camera.cameraClose(_camera)
        relay.shutdown()
        closeSerial()