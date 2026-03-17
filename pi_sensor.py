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

# ---------------------------------------------------------------
# OPTIONAL LIBRARIES
# ---------------------------------------------------------------
# These imports match the handout/starter naming. If your local
# helper files use slightly different names, only these imports /
# calls need minor adjustment.

try:
    from alex_camera import cameraOpen, cameraClose, captureGreyscaleFrame, renderGreyscaleFrame
except Exception:
    cameraOpen = cameraClose = captureGreyscaleFrame = renderGreyscaleFrame = None

try:
    from lidar.alex_lidar import AlexLidar
except Exception:
    AlexLidar = None

try:
    from lidar_example_cli_plot import plot_single_scan
except Exception:
    plot_single_scan = None


# ---------------------------------------------------------------
# SERIAL PORT SETUP
# ---------------------------------------------------------------

PORT = "/dev/ttyACM0"
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


# ---------------------------------------------------------------
# TPACKET CONSTANTS
# (must match packets.h / Arduino code)
# ---------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP = 0
COMMAND_COLOR = 1

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOR  = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1  # 103


# ---------------------------------------------------------------
# RELIABLE FRAMING: MAGIC + XOR CHECKSUM
# ---------------------------------------------------------------

def computeChecksum(data: bytes) -> int:
    """Return XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.
    """
    if params is None:
        params = [0] * PARAMS_COUNT

    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command, data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command': fields[1],
        'data': fields[2],
        'params': list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes until a valid framed packet is found.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        b = _ser.read(1)
        if not b:
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
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ---------------------------------------------------------------
# E-STOP STATE
# ---------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active."""
    return _estop_state == STATE_STOPPED


# ---------------------------------------------------------------
# PACKET DISPLAY
# ---------------------------------------------------------------

def printPacket(pkt):
    """
    Print a received TPacket in human-readable form.
    """
    global _estop_state

    ptype = pkt['packetType']
    cmd = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")

        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
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


# ---------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ---------------------------------------------------------------

def handleColorCommand():
    """
    Request a color reading from the Arduino and display it.
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    print("Requesting color reading...")
    sendCommand(COMMAND_COLOR)


# ---------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ---------------------------------------------------------------

_camera = None
_frames_remaining = 5


def handleCameraCommand():
    """
    Capture and display one greyscale frame.
    """
    global _frames_remaining

    if isEstopActive(): 
        print("Refused: E-Stop is active")
        return

    if _frames_remaining <= 0:
        print("Refused: No camera frames remaining")
        return

    if _camera is None:
        print("Camera not available.")
        return

    try:
        frame = captureGreyscaleFrame(_camera)
        renderGreyscaleFrame(frame)
        _frames_remaining -= 1
        print(f"Frames remaining: {_frames_remaining}")
    except Exception as e:
        print(f"Camera capture failed: {e}")


# ---------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ---------------------------------------------------------------

_lidar = None


def ensureLidarOpen():
    global _lidar

    if _lidar is not None:
        return True

    if AlexLidar is None:
        print("LIDAR library not available.")
        return False

    try:
        _lidar = AlexLidar()
        return True
    except Exception as e:
        print(f"LIDAR open failed: {e}")
        return False


def handleLidarCommand():
    """
    Perform a single LIDAR scan and render it.
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    if plot_single_scan is None:
        print("LIDAR plot helper not available.")
        return

    if not ensureLidarOpen():
        return

    try:
        # This matches the usual starter usage from the LIDAR studio.
        scan = _lidar.get_single_scan()
        plot_single_scan(scan)
    except AttributeError:
        print("Your AlexLidar API may use a different method name than get_single_scan().")
    except Exception as e:
        print(f"LIDAR scan failed: {e}")


# ---------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ---------------------------------------------------------------

def handleUserInput(line):
    """
    Dispatch user input.
    """
    cmd = line[0]

    if cmd == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')

    elif cmd == 'c':
        handleColorCommand()

    elif cmd == 'p':
        handleCameraCommand()

    elif cmd == 'l':
        handleLidarCommand()

    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l")


def runCommandInterface():
    """
    Main command loop.
    """
    print("Sensor interface ready. Type e / c / p / l and press Enter.")
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


# ---------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------

if __name__ == '__main__':
    openSerial()

    if cameraOpen is not None:
        try:
            _camera = cameraOpen()
            print("Camera ready.")
        except Exception as e:
            print(f"Camera open failed at startup: {e}")

    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        try:
            if _camera is not None and cameraClose is not None:
                cameraClose(_camera)
        except Exception:
            pass

        try:
            if _lidar is not None:
                # Use whichever close/disconnect method your library provides.
                if hasattr(_lidar, 'disconnect'):
                    _lidar.disconnect()
                elif hasattr(_lidar, 'stop'):
                    _lidar.stop()
        except Exception:
            pass

        closeSerial()
