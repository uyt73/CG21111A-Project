#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py
"""

import struct
import serial
import time
import sys
import select

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None

def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")

def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()

# ----------------------------------------------------------------
# TPACKET CONSTANTS
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP       = 0
COMMAND_CLEAR_ESTOP = 1
COMMAND_FORWARD     = 2
COMMAND_BACKWARD    = 3
COMMAND_TURN_LEFT   = 4
COMMAND_TURN_RIGHT  = 5
COMMAND_SPEED_UP    = 6
COMMAND_SPEED_DOWN  = 7
COMMAND_STOP        = 8
COMMAND_COLOR       = 10 # Activity 2: Color Sensor

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOR  = 11     # Activity 2: Color Sensor Response

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103

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
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]
    
    discarded = 0 # The Escape Hatch Counter

    while True:
        b = _ser.read(1)
        if not b:
            return None          
            
        # If we read 200 bytes of garbage without syncing, bail out to check keyboard
        discarded += 1
        if discarded > 200:
            return None

        if b[0] != MAGIC_HI:
            continue

        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            continue

        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            continue

        return unpackTPacket(raw)

def sendCommand(commandType, data=b'', params=None):
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)

# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING

def isEstopActive():
    return _estop_state == STATE_STOPPED

# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            
            if _estop_state == STATE_STOPPED and state == STATE_RUNNING:
                timestamp = time.strftime('%H:%M:%S')
                print(f"\n[{timestamp}] LOG: E-Stop Cleared. Hardware is ENABLED.")
                
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")
                
        elif cmd == RESP_COLOR:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"Color: R={r} Hz, G={g} Hz, B={b} Hz")
        else:
            print(f"Response: unknown command {cmd}")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")

# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
        
    print("Sending color command...")
    sendCommand(COMMAND_COLOR)

# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

import alex_camera

print("Initializing camera...")
_camera = alex_camera.cameraOpen() 
_frames_remaining = 5

def handleCameraCommand():
    global _frames_remaining

    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
        
    if _frames_remaining <= 0:
        print("Refused: No camera frames remaining.")
        return
    
    print("Capturing image...")
    frame = alex_camera.captureGreyscaleFrame(_camera)
    alex_camera.renderGreyscaleFrame(frame)

    _frames_remaining -= 1
    print(f"Frames remaining: {_frames_remaining}")

# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

from lidar import alex_lidar
import lidar_example_cli_plot

def handleLidarCommand():
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
        
    print("Starting LIDAR scan...")
    lidar_example_cli_plot.plot_single_scan()

# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

def handleUserInput(line):
    if line == 'r':
        print("Clearing E-Stop state...")
        sendCommand(COMMAND_CLEAR_ESTOP)
    elif line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    elif line == 'c':
        handleColorCommand()
    elif line == 'p':
        handleCameraCommand()
    elif line == 'l':
        handleLidarCommand()
        
    # Movement Commands (Optional, but useful for the final integration)
    elif line == 'w':
        sendCommand(COMMAND_FORWARD)
    elif line == 's':
        sendCommand(COMMAND_BACKWARD)
    elif line == 'a':
        sendCommand(COMMAND_TURN_LEFT)
    elif line == 'd':
        sendCommand(COMMAND_TURN_RIGHT)
    elif line == '+':
        sendCommand(COMMAND_SPEED_UP)
    elif line == '-':
        sendCommand(COMMAND_SPEED_DOWN)
    elif line == 'h':
        print("Stopping Robot")
        sendCommand(COMMAND_STOP)
        
    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l, w, a, s, d, +, -, h")

def runCommandInterface():
    print("Sensor interface ready. Type e / c / p / l / w / a / s / d and press Enter.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)

        time.sleep(0.05)

# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if _camera is not None:
            alex_camera.cameraClose(_camera)
        closeSerial()