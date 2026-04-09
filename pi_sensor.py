#!/usr/bin/env python3
"""
Studio 15: Robot Assembly & Remote Control
Updated pi_sensor.py
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
    discarded = 0 
    while True:
        b = _ser.read(1)
        if not b: return None 
        discarded += 1
        if discarded > 200: return None
        if b[0] != MAGIC_HI: continue
        b = _ser.read(1)
        if not b: return None
        if b[0] != MAGIC_LO: continue
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk: return None
            raw += chunk
        cs_byte = _ser.read(1)
        if not cs_byte: return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected: continue
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
            # If params[0] is not 0, it likely contains the new Speed value
            if pkt['params'][0] > 0:
                print(f"Response: OK (Current Speed: {pkt['params'][0]})")
            else:
                print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            if _estop_state == STATE_STOPPED and state == STATE_RUNNING:
                print(f"\n[{time.strftime('%H:%M:%S')}] LOG: E-Stop Cleared. Systems Active.")
            _estop_state = state
            print("Status: RUNNING" if state == STATE_RUNNING else "Status: STOPPED")
        elif cmd == RESP_COLOR:
            print(f"Color: R={pkt['params'][0]} Hz, G={pkt['params'][1]} Hz, B={pkt['params'][2]} Hz")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug: print(f"Arduino debug: {debug}")

# ----------------------------------------------------------------
# HANDLERS
# ----------------------------------------------------------------

def handleColorCommand():
    if isEstopActive(): print("Refused: E-Stop Active."); return
    sendCommand(COMMAND_COLOR)

# (Lidar/Camera handlers omitted for brevity - keep your existing ones)

# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

def handleUserInput(line):
    # Safety Gate for all movement commands
    if line in ['w', 's', 'a', 'd', 'h', '+', '-'] and isEstopActive():
        print("Refused: E-Stop is active. Press 'r' to clear.")
        return

    if line == 'r':
        print("Clearing E-Stop state...")
        sendCommand(COMMAND_CLEAR_ESTOP)
    elif line == 'e':
        sendCommand(COMMAND_ESTOP)
    elif line == 'c':
        handleColorCommand()
    elif line == 'w': sendCommand(COMMAND_FORWARD)
    elif line == 's': sendCommand(COMMAND_BACKWARD)
    elif line == 'a': sendCommand(COMMAND_TURN_LEFT)
    elif line == 'd': sendCommand(COMMAND_TURN_RIGHT)
    elif line == '+': sendCommand(COMMAND_SPEED_UP)
    elif line == '-': sendCommand(COMMAND_SPEED_DOWN)
    elif line == 'h': sendCommand(COMMAND_STOP)
    else:
        print(f"Unknown input: '{line}'")

def runCommandInterface():
    # UPDATED MENU LINE
    print("Controls: [w/s/a/d] Move | [h] Stop | [+/-] Speed | [c/p/l] Sensors | [e] E-Stop | [r] Reset")
    print("Press Ctrl+C to exit.\n")

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
        closeSerial()