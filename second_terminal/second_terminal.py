#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Second operator terminal (Arm Control).

This terminal connects to pi_sensor.py over TCP.  It:
  - Displays every TPacket forwarded from the robot (via pi_sensor.py).
  - Handles the 4-DOF Robotic Arm controls.
  - Sends a software E-Stop command when you type 'e'.

Run pi_sensor.py FIRST (it starts the TCP server), then run this script.
"""

import select
import struct
import sys
import time
import os

# ---------------------------------------------------------------------------
# Import Shared Packets
# ---------------------------------------------------------------------------
# Add parent directory to path so we can import packets.py
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from packets import *

# net_utils is imported with an absolute import
from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame

# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------
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
            pass # Keep terminal clean, only print explicit logs
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
# Input handling (The Payload Operator)
# ---------------------------------------------------------------------------

def _handleInput(line: str, client: TCPClient):
    line = line.strip().lower()
    if not line: return

    # --- Safety Gate ---
    if line in ['o', 'c', ',', '.', 'u', 'j', 'i', 'k'] and _estop_active:
        print("[second_terminal] Refused: E-Stop is active.")
        return

    # --- Core Commands ---
    if line == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")
    elif line == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    # --- 4-DOF Arm Commands ---
    elif line == 'o': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_GRIPPER_OPEN))
        print("[arm] Gripper: Open")
    elif line == 'c': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_GRIPPER_CLOSE))
        print("[arm] Gripper: Close")
    
    elif line == ',': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_BASE_LEFT))
        print("[arm] Base: Left")
    elif line == '.': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_BASE_RIGHT))
        print("[arm] Base: Right")
        
    elif line == 'u': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_SHOULDER_UP))
        print("[arm] Shoulder: Up")
    elif line == 'j': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_SHOULDER_DOWN))
        print("[arm] Shoulder: Down")
        
    elif line == 'i': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_ELBOW_UP))
        print("[arm] Elbow: Up")
    elif line == 'k': 
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, COMMAND_ELBOW_DOWN))
        print("[arm] Elbow: Down")
        
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
    print("  [o/c] Gripper | [,/.] Base | [u/j] Shoulder | [i/k] Elbow")
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