#include "packets.h"
#include "serial_driver.h"

int currentSpeed = 150; // Variable to store speed [cite: 221]

void setup() {
  // Serial setup from Studio 13 
#if USE_BAREMETAL_SERIAL
  usartInit(103); 
#else
  Serial.begin(9600);
#endif

  sei(); 
}

// Wrapper functions to call robotlib.ino [cite: 215, 15]
void driveForward()  { forward(currentSpeed); }
void driveBackward() { backward(currentSpeed); }
void turnLeft()      { ccw(currentSpeed); }
void turnRight()     { cw(currentSpeed); }

void changeSpeed(int delta) {
  currentSpeed += delta;
  if (currentSpeed > 255) currentSpeed = 255;
  if (currentSpeed < 0) currentSpeed = 0;
}

void handleCommand(const TPacket *cmd) {
  if (cmd->packetType != PACKET_TYPE_COMMAND) return;

  switch (cmd->command) {
    case COMMAND_FORWARD:    driveForward();  break;
    case COMMAND_BACKWARD:   driveBackward(); break;
    case COMMAND_TURN_LEFT:  turnLeft();      break;
    case COMMAND_TURN_RIGHT: turnRight();     break;
    case COMMAND_SPEED_UP:   changeSpeed(25); break;
    case COMMAND_SPEED_DOWN: changeSpeed(-25);break;
    case COMMAND_ESTOP:      stop();          break;
    case COMMAND_STOP:       stop();          break;
  }
}

void loop() {
  TPacket incoming;
  // Instead of hardcoded delays, we check for new serial packets 
  if (receiveFrame(&incoming)) {
    handleCommand(&incoming);
  }
}