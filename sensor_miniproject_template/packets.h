/*
 * packets.h
 * Studio 16: Robot Integration (4-DOF Arm)
 */

#pragma once

#include <stdint.h>

// =============================================================
// TPacket protocol
// =============================================================

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    COMMAND_ESTOP       = 0,
    COMMAND_CLEAR_ESTOP = 1,
    COMMAND_FORWARD     = 2,
    COMMAND_BACKWARD    = 3,
    COMMAND_TURN_LEFT   = 4,
    COMMAND_TURN_RIGHT  = 5,
    COMMAND_SPEED_UP    = 6,
    COMMAND_SPEED_DOWN  = 7,
    COMMAND_STOP        = 8,
    COMMAND_COLOR       = 10,

    // --- 4-DOF Arm Commands ---
    COMMAND_GRIPPER_OPEN  = 12,
    COMMAND_GRIPPER_CLOSE = 13,
    COMMAND_SET_BASE      = 14,
    COMMAND_SET_SHOULDER  = 15,
    COMMAND_SET_ELBOW     = 16
} TCommandType;

typedef enum {
    RESP_OK     = 0,
    RESP_STATUS = 1,
    RESP_COLOR  = 11
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    char     data[32];
    uint32_t params[16];
} TPacket;

// =============================================================
// Framing constants
// =============================================================

#define MAGIC_HI        0xDE
#define MAGIC_LO        0xAD
#define TPACKET_SIZE    ((uint8_t)sizeof(TPacket))   // 100 bytes
#define FRAME_SIZE      (2 + TPACKET_SIZE + 1)       // 103 bytes