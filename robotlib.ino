#include <AFMotor.h>

// Direction values
typedef enum dir {
  STOP,
  GO,
  BACK,
  CCW,
  CW
} dir;

// Motor control pins
#define FRONT_LEFT   4 // M4
#define FRONT_RIGHT  1 // M1
#define BACK_LEFT    3 // M3
#define BACK_RIGHT   2 // M2

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

// =======================================================================
// THE SOFTWARE POLARITY MATRIX
// If a wheel spins backward when you press 'w', swap FORWARD and BACKWARD here!
// =======================================================================
#define FL_FWD FORWARD
#define FL_BWD BACKWARD 

#define FR_FWD FORWARD  
#define FR_BWD BACKWARD 

#define BL_FWD BACKWARD  
#define BL_BWD FORWARD

#define BR_FWD FORWARD 
#define BR_BWD BACKWARD 
// =======================================================================

void move(int speed, int direction) {
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);

  switch(direction) {
    case GO: // All wheels go forward
      motorFL.run(FL_FWD);
      motorFR.run(FR_FWD);
      motorBL.run(BL_FWD);
      motorBR.run(BR_FWD); 
      break;
      
    case BACK: // All wheels go backward
      motorFL.run(FL_BWD);
      motorFR.run(FR_BWD);
      motorBL.run(BL_BWD);
      motorBR.run(BR_BWD); 
      break;
      
    case CW: // Right Turn: Left wheels forward, Right wheels backward
      motorFL.run(FL_FWD);
      motorFR.run(FR_BWD);
      motorBL.run(BL_FWD);
      motorBR.run(BR_BWD); 
      break;
      
    case CCW: // Left Turn: Left wheels backward, Right wheels forward
      motorFL.run(FL_BWD);
      motorFR.run(FR_FWD);
      motorBL.run(BL_BWD);
      motorBR.run(BR_FWD); 
      break;
      
    case STOP:
    default:
      motorFL.run(RELEASE);
      motorFR.run(RELEASE);
      motorBL.run(RELEASE);
      motorBR.run(RELEASE); 
  }
}

void forward(int speed)  { move(speed, GO); }
void backward(int speed) { move(speed, BACK); }
void ccw(int speed)      { move(speed, CCW); }
void cw(int speed)       { move(speed, CW); }
void stop()              { move(0, STOP); }