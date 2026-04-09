#!/usr/bin/env python3
"""
CG2111A Alex Robot - Integrated Pi Interface
Combines Studio 13 (Sensors) and Studio 15 (Movement)
"""

import struct
import serial
import time
import sys
import select
import alex_camera
from lidar import alex_lidar
import lidar_example_cli_plot

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
        _ser.close()

# ----------------------------------------------------------------
# TPACKET CONSTANTS (Must match Arduino packets.h)
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
COMMAND_COLOR       = 10

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOR  = 11

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4) 
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1 

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
    MAGIC_HI, MAGIC_LO = MAGIC[0], MAGIC[1]
    discarded = 0 
    while True:
        b = _ser.read(1)
        if not b: return None 
        discarded += 1
        if discarded > 200: return None # Escape hatch for garbage data
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

def sendCommand(commandType, data=b'', params=None):
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)

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
            if pkt['params'][0] > 0: # Speed updates return current speed
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
_frames_remaining = 5

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

def handleLidarCommand():
    if isEstopActive(): print("Refused: E-Stop Active."); return
    lidar_example_cli_plot.plot_single_scan()

# ----------------------------------------------------------------
# MAIN INTERFACE
# ----------------------------------------------------------------
def handleUserInput(line):
    # Safety gate for movement/sensors
    if line in ['w','s','a','d','h','+','-','c','p','l'] and isEstopActive():
        print("Refused: E-Stop is active. Press 'r' to reset.")
        return

    if line == 'r': sendCommand(COMMAND_CLEAR_ESTOP)
    elif line == 'e': sendCommand(COMMAND_ESTOP)
    elif line == 'c': handleColorCommand()
    elif line == 'p': handleCameraCommand()
    elif line == 'l': handleLidarCommand()
    elif line == 'w': sendCommand(COMMAND_FORWARD)
    elif line == 's': sendCommand(COMMAND_BACKWARD)
    elif line == 'a': sendCommand(COMMAND_TURN_LEFT)
    elif line == 'd': sendCommand(COMMAND_TURN_RIGHT)
    elif line == '+': sendCommand(COMMAND_SPEED_UP)
    elif line == '-': sendCommand(COMMAND_SPEED_DOWN)
    elif line == 'h': sendCommand(COMMAND_STOP)
    else: print(f"Unknown command: '{line}'")

def runCommandInterface():
    print("Controls: [w/s/a/d] Move | [h] Stop | [+/-] Speed | [c/p/l] Sensors | [e] E-Stop | [r] Reset")
    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt: printPacket(pkt)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if line: handleUserInput(line)
        time.sleep(0.05)

if __name__ == '__main__':
    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if _camera: alex_camera.cameraClose(_camera)
        closeSerial()