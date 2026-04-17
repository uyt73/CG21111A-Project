#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Real-Time Arm Control Pad (Calibrator Style)
"""

import select
import struct
import sys
import time
import os
import tty
import termios

# --- Import Shared Packets ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from packets import *

from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame

PI_HOST = 'localhost'
PI_PORT = 65432

# ---------------------------------------------------------------------------
# State Memory (Matches Arduino Defaults)
# ---------------------------------------------------------------------------
angles = {'b': 90, 's': 0, 'l': 90, 'g': 5}
active_joint = 'g'
joint_names = {'b': 'Base', 's': 'Shoulder', 'l': 'Elbow', 'g': 'Gripper'}
_estop_active = False

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
# UI and Output Handling
# ---------------------------------------------------------------------------
def _printStatus():
    """Prints the live updating status bar at the bottom of the terminal."""
    status = f"\rActive: {joint_names[active_joint]:<8} | Angle: [{angles[active_joint]:03d}°] | E-Stop: {'ON ' if _estop_active else 'OFF'} "
    sys.stdout.write(status)
    sys.stdout.flush()

def _printPacket(pkt):
    global _estop_active
    ptype = pkt['packetType']
    cmd   = pkt['command']

    # Clear the current status line before printing a new log message
    sys.stdout.write("\r" + " "*60 + "\r") 

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
            
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")
        
    _printStatus() # Redraw the live status bar

# ---------------------------------------------------------------------------
# Instant Input Handling
# ---------------------------------------------------------------------------
def _handleChar(ch: str, client: TCPClient):
    global active_joint

    # 1. Select Active Joint
    if ch in ['b', 's', 'l', 'g']:
        active_joint = ch
        _printStatus()
        return

    # 2. Safety Gate
    if ch in ['w', 'a', 's', 'd'] and _estop_active:
        return # Ignore movement if E-Stop is active

    # 3. Core Commands
    if ch == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        return

    # 4. Angle Manipulation (w/s/a/d)
    changed = False
    if ch == 'w':   # +1 degree
        angles[active_joint] += 1
        changed = True
    elif ch == 's': # -1 degree
        angles[active_joint] -= 1
        changed = True
    elif ch == 'd': # +5 degrees
        angles[active_joint] += 5
        changed = True
    elif ch == 'a': # -5 degrees
        angles[active_joint] -= 5
        changed = True

    if changed:
        # Enforce physical hardware limits
        if active_joint == 'g':
            angles[active_joint] = max(0, min(30, angles[active_joint]))
        else:
            angles[active_joint] = max(0, min(180, angles[active_joint]))

        # Map character to the specific packet command
        cmd_map = {
            'b': COMMAND_SET_BASE, 
            's': COMMAND_SET_SHOULDER, 
            'l': COMMAND_SET_ELBOW, 
            'g': COMMAND_SET_GRIPPER
        }
        
        # Send the new angle
        params = [0] * PARAMS_COUNT
        params[0] = angles[active_joint]
        sendTPacketFrame(client.sock, _packFrame(PACKET_TYPE_COMMAND, cmd_map[active_joint], params=params))
        
        _printStatus()

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def run():
    client = TCPClient(host=PI_HOST, port=PI_PORT)
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=60.0):
        print("[second_terminal] Could not connect. Ensure pi_sensor.py is running.")
        sys.exit(1)

    print("\n" + "="*40)
    print("     PAYLOAD OPERATOR ACTIVE")
    print("="*40)
    print("Controls:")
    print(" [g, b, s, l] : Select active joint")
    print(" [w] / [s]    : Nudge angle UP/DOWN by 1 deg")
    print(" [a] / [d]    : Jump  angle DOWN/UP by 5 deg")
    print(" [e]          : Emergency Stop")
    print(" [q]          : Quit")
    print("="*40 + "\n")

    # Capture the terminal state so we can restore it when we quit
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # Set the terminal to "cbreak" mode (instant character read, but allows Ctrl+C to kill it)
        tty.setcbreak(fd)
        _printStatus()

        while True:
            # Check for incoming packets from the robot
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    sys.stdout.write("\r\n[second_terminal] Connection closed.\n")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            # Check for instant keystrokes from the operator
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1).lower()
                if ch == 'q':
                    raise KeyboardInterrupt
                elif ch:
                    _handleChar(ch, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        sys.stdout.write("\r\n[second_terminal] Exiting.\n")
    finally:
        # Crucial: Return the terminal to normal mode, or else the text will break when you exit!
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        client.close()

if __name__ == '__main__':
    run()