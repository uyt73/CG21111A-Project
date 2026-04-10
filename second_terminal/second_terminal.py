#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Second operator terminal (Arm Control).
"""

import select
import struct
import sys
import time
import os

# --- Import Shared Packets ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from packets import *

from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame

PI_HOST = 'localhost'
PI_PORT = 65432

# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result

def _packFrame(packetType, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded  = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])

def _unpackFrame(frame: bytes):
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }

# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False

def _printPacket(pkt):
    global _estop_active
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            pass 
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        else:
            print(f"[robot] Response: command {cmd} finished.")
            
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

def _handleInput(line: str, client: TCPClient):
    line = line.strip().lower()
    if not line: return

    parts = line.split()
    cmd_char = parts[0]

    # --- Safety Gate ---
    if cmd_char in ['o', 'c', 'b', 's', 'e'] and _estop_active:
        print("[second_terminal] Refused: E-Stop is active.")
        return

    # --- Core Commands ---
    if cmd_char == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")
    elif cmd_char == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    # --- Gripper Commands (Open/Close) ---
    elif cmd_char == 'o': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_GRIPPER_OPEN))
        print("[arm] Gripper: Open")
    elif cmd_char == 'c': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_GRIPPER_CLOSE))
        print("[arm] Gripper: Close")
    
    # --- Absolute Angle Commands (Base, Shoulder, Elbow) ---
    elif cmd_char in ['b', 's', 'e']:
        if len(parts) < 2:
            print(f"[second_terminal] Error: Provide an angle (e.g., '{cmd_char} 90')")
            return
        
        try:
            angle = int(parts[1])
        except ValueError:
            print("[second_terminal] Error: Angle must be an integer.")
            return

        # Load the angle into the first parameter slot
        params = [0] * PARAMS_COUNT
        params[0] = angle

        if cmd_char == 'b':
            sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_SET_BASE, params=params))
            print(f"[arm] Base set to {angle} deg")
        elif cmd_char == 's':
            sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_SET_SHOULDER, params=params))
            print(f"[arm] Shoulder set to {angle} deg")
        elif cmd_char == 'e':
            sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_SET_ELBOW, params=params))
            print(f"[arm] Elbow set to {angle} deg")
            
    else:
        print(f"[second_terminal] Unknown: '{line}'.")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run():
    client = TCPClient(host=PI_HOST, port=PI_PORT)
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=60.0):
        print("[second_terminal] Could not connect. Ensure pi_sensor.py is running.")
        sys.exit(1)

    print("\n[second_terminal] Connected! --- PAYLOAD OPERATOR ACTIVE ---")
    print("Controls:")
    print("  [o/c] Gripper | [b <angle>] Base | [s <angle>] Shoulder | [e <angle>] Elbow")
    print("  Example: 's 120' sets the shoulder to 120 degrees.")
    print("  [e] E-Stop    | [q] Quit\n")

    try:
        while True:
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("[second_terminal] Connection closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        client.close()

if __name__ == '__main__':
    run()