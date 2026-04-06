#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select
import second_terminal.relay as relay


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
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------

from packets import *

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            # Not the magic number; keep searching (don't skip the byte
            # we just read in case it is the first byte of another frame).
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form.

    The 'data' field carries an optional debug string from the Arduino.
    When non-empty, it is printed automatically so you can embed debug
    messages in any outgoing TPacket on the Arduino side (set pkt.data to
    a null-terminated string up to 31 characters before calling sendFrame).
    This works like Serial.print(), but sends output to the Pi terminal.
    """
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
                print(f"\n[{timestamp}] LOG: E-Stop Cleared. Robot movement is now ENABLED.")

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
        # Print the optional debug string from the data field.
        # On the Arduino side, fill pkt.data before calling sendFrame() to
        # send debug messages to this terminal (similar to Serial.print()).
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
    """
    TODO (Activity 2): request a color reading from the Arduino and display it.

    Check the E-Stop state first; if stopped, refuse with a clear message.
    Otherwise, send your color command to the Arduino.
    """
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
        
    print("Sending color command...")
    sendCommand(COMMAND_COLOR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

import alex_camera

# Open the camera and store the reference
print("Initializing camera...")
_camera = alex_camera.cameraOpen() 
_frames_remaining = 5


def handleCameraCommand():
    """
    TODO (Activity 3): capture and display a greyscale frame.
    Gate on E-Stop state and the remaining frame count.
    """
    global _frames_remaining

    # 1. Enforce the E-Stop gate
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
        
    # 2. Enforce the frame limit (max 5 pictures)
    if _frames_remaining <= 0:
        print("Refused: No camera frames remaining.")
        return
    
    # 3. Capture and display the photo
    print("Capturing image...")
    frame = alex_camera.captureGreyscaleFrame(_camera)
    alex_camera.renderGreyscaleFrame(frame)

    # 4. Decrease the counter and print
    _frames_remaining -= 1
    print(f"Frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel

# Map size and resolution settings
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 32   # 32x32 meter map

from lidar import alex_lidar
import lidar_example_cli_plot


def handleLidarCommand():
    """
    TODO (Activity 4): perform a single LIDAR scan and render it.

    Gate on E-Stop state, then use the LIDAR library to capture one scan
    and the CLI plot helpers to display it.
    """
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
        
    print("Starting LIDAR scan...")

    lidar_example_cli_plot.plot_single_scan()

def generateSlamMap():
    """
    Runs a continuous SLAM loop. 
    You can still drive the robot using a second terminal or predefined commands.
    """
    if isEstopActive():
        print("REFUSED: E-Stop is active.")
        return

    print("\n--- INITIALIZING SLAM ---")
    print("Please wait, spinning up LIDAR...")
    
    # 1. Initialize SLAM with the LIDAR model and map dimensions
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
    
    # 2. Start the LIDAR motor
    alex_lidar.start_motor()
    time.sleep(2) # Give the motor time to reach full speed

    print("Mapping started! Drive the robot slowly.")
    print("Press Ctrl+C to STOP mapping and save the image.\n")

    try:
        scan_count = 0
        # 3. Continuous mapping loop
        for scan in alex_lidar.iter_scans():
            # Extract distances (in mm) from the scan data
            # Typically iter_scans yields tuples of (quality, angle, distance)
            distances = [item[2] for item in scan]

            # 4. Update the SLAM algorithm with the new distances
            # (We pad or truncate to match the RPLidarA1's expected 360 points)
            if len(distances) >= 360:
                slam.update(distances[:360])
                scan_count += 1
                
                if scan_count % 10 == 0:
                    x, y, theta = slam.getpos()
                    print(f"Scans processed: {scan_count} | Robot Pos: X={x/1000:.2f}m, Y={y/1000:.2f}m")

    except KeyboardInterrupt:
        print("\nMapping interrupted by user. Generating map file...")
    
    finally:
        # 5. Stop the hardware and extract the map
        alex_lidar.stop_motor()
        
        # Get the map bytes (0-255 grayscale values)
        map_bytes = slam.getmap()
        
        # Save as a PGM image file (can be opened by most image viewers)
        filename = f"moonbase_map_{int(time.time())}.pgm"
        with open(filename, 'wb') as f:
            f.write(f"P5\n{MAP_SIZE_PIXELS} {MAP_SIZE_PIXELS}\n255\n".encode())
            f.write(map_bytes)
            
        print(f"SUCCESS: Map saved to {filename}")

# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

# User input -> action mapping:
#   e  send a software E-Stop command to the Arduino (pre-wired)
#   c  request color reading from the Arduino        (Activity 2 - implement yourself)
#   p  capture and display a camera frame            (Activity 3 - implement yourself)
#   l  perform a single LIDAR scan                   (Activity 4 - implement yourself)


def handleUserInput(line):
    """
    Dispatch a single line of user input.

    The 'e' case is pre-wired to send a software E-Stop command.
    TODO (Activities 2, 3 & 4): add 'c' (color), 'p' (camera) and 'l' (LIDAR).
    """
    if line == 'r':
        print("Clearing E-Stop state...")
        sendCommand(COMMAND_CLEAR_ESTOP)
        return

    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    elif line == 'c':
        handleColorCommand()
    elif line == 'p':
        handleCameraCommand()
    elif line == 'l':
        handleLidarCommand()
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
    elif line == 'm':
        generateSlamMap()
    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l, w, a, s, d, +, -")


def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """
    print("Sensor interface ready. Type e / c / p / l / w / a / s / d / + / - and press Enter.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'],pkt['data'], pkt['params']))
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)
        relay.checkSecondTerminal(_ser)
        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    relay.start()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if _camera is not None:
            alex_camera.cameraClose(_camera)
        relay.shutdown()
        closeSerial()