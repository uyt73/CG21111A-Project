/*
 * sensor_miniproject_template.ino
 * Studio 16: Robot Integration 
 * Features: 500ms Heartbeat, Polarity Matrix, Turn Boost, Bare-Metal 4-DOF Arm, Max Speed Hardcoded
 */

#include "packets.h"
#include "serial_driver.h"
#include <AFMotor.h>

// --- HARDCODED MAX SPEED ---
const int robotSpeed = 255; 

// --- Heartbeat Timeout Variables ---
unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT = 500; // Drives for 0.5 seconds per keypress

// =============================================================
// DC Motor Setup & Software Polarity Matrix
// =============================================================
typedef enum dir {
  DIR_STOP,
  DIR_GO,
  DIR_BACK,
  DIR_CCW,
  DIR_CW
} dir_t;

// Motor control pins
#define FRONT_LEFT   4 // M4
#define FRONT_RIGHT  1 // M1
#define BACK_LEFT    3 // M3
#define BACK_RIGHT   2 // M2

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

// THE SOFTWARE POLARITY MATRIX
// If a wheel spins backward when you press 'w', swap FORWARD and BACKWARD here!
#define FL_FWD BACKWARD
#define FL_BWD FORWARD

#define FR_FWD BACKWARD  
#define FR_BWD FORWARD

#define BL_FWD BACKWARD  
#define BL_BWD FORWARD

#define BR_FWD BACKWARD  
#define BR_BWD FORWARD

void move(int speed, dir_t direction) {
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);

  switch(direction) {
    case DIR_GO: // All wheels go forward
      motorFL.run(FL_FWD);
      motorFR.run(FR_FWD);
      motorBL.run(BL_FWD);
      motorBR.run(BR_BWD); 
      break;
      
    case DIR_BACK: // All wheels go backward
      motorFL.run(FL_BWD);
      motorFR.run(FR_BWD);
      motorBL.run(BL_BWD);
      motorBR.run(BR_FWD); 
      break;
      
    case DIR_CW: // Right Turn: Left wheels forward, Right wheels backward
      motorFL.run(FL_BWD);
      motorFR.run(FR_BWD);
      motorBL.run(BL_FWD);
      motorBR.run(BR_BWD); 
      break;
      
    case DIR_CCW: // Left Turn: Left wheels backward, Right wheels forward
      motorFL.run(FL_FWD);
      motorFR.run(FR_FWD);
      motorBL.run(BL_BWD);
      motorBR.run(BR_FWD); 
      break;
      
    case DIR_STOP:
    default:
      motorFL.run(RELEASE);
      motorFR.run(RELEASE);
      motorBL.run(RELEASE);
      motorBR.run(RELEASE); 
  }
}

void forward(int speed)  { move(speed, DIR_GO); }
void backward(int speed) { move(speed, DIR_BACK); }

// Turn Boost (caps safely at 255)
void ccw(int speed)      { 
    int turnSpeed = speed + 60;
    if (turnSpeed > 255) turnSpeed = 255;
    move(turnSpeed, DIR_CCW); 
}
void cw(int speed)       { 
    int turnSpeed = speed + 60;
    if (turnSpeed > 255) turnSpeed = 255;
    move(turnSpeed, DIR_CW); 
}
void stop()              { move(0, DIR_STOP); }


// =============================================================
// Bare-Metal Servo Driver (Timer 5)
// Base (Pin 49 / PL0), Shoulder (Pin 9 / PH6)
// Elbow (Pin 10 / PB4), Gripper (Pin 51 / PB2)
// =============================================================

volatile uint16_t servoPulses[4] = {3000, 3000, 3000, 3000}; // Default 90 deg (1500us * 2 ticks)

int baseAngle     = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int gripperAngle  = 15; // Set starting angle within the safe 0-30 range

void bareMetalServoInit() {
    // 1. Set pins as outputs
    DDRL |= (1 << PL0);  // Pin 49
    DDRH |= (1 << PH6);  // Pin 9
    DDRB |= (1 << PB4);  // Pin 10
    DDRB |= (1 << PB2);  // Pin 51

    // 2. Configure Timer 5 for CTC (Clear Timer on Compare Match)
    cli();
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5  = 0;
    
    TCCR5B |= (1 << WGM52); // Enable CTC Mode
    TCCR5B |= (1 << CS51);  // Prescaler 8 -> 2MHz timer (0.5us per tick)
    
    OCR5A = 1000;           // Start first interrupt quickly
    TIMSK5 |= (1 << OCIE5A); // Enable interrupt
    sei();
}

// Timer 5 Interrupt Service Routine: Fires sequentially to pulse each servo
ISR(TIMER5_COMPA_vect) {
    static uint8_t servoIndex = 0;
    static uint16_t totalTicks = 0;

    // Turn OFF the previous servo pin
    if (servoIndex == 1)      PORTL &= ~(1 << PL0);
    else if (servoIndex == 2) PORTH &= ~(1 << PH6);
    else if (servoIndex == 3) PORTB &= ~(1 << PB4);
    else if (servoIndex == 4) PORTB &= ~(1 << PB2);

    // Turn ON the current servo pin and set its duration
    if (servoIndex < 4) {
        if (servoIndex == 0)      PORTL |= (1 << PL0);
        else if (servoIndex == 1) PORTH |= (1 << PH6);
        else if (servoIndex == 2) PORTB |= (1 << PB4);
        else if (servoIndex == 3) PORTB |= (1 << PB2);

        OCR5A = servoPulses[servoIndex];
        totalTicks += servoPulses[servoIndex];
        servoIndex++;
    } 
    // All 4 servos pulsed. Pad the rest of the 20ms (40,000 ticks) window
    else {
        if (totalTicks < 40000) {
            OCR5A = 40000 - totalTicks;
        } else {
            OCR5A = 1000; // Fallback
        }
        servoIndex = 0;
        totalTicks = 0;
    }
}

void setServoAngle(uint8_t index, int angle) {
    // --- Custom Mechanical Safety Limits ---
    if (index == 3) {
        // Gripper: Restricted to 0 -> 30 degrees
        if (angle < 0) angle = 0;
        if (angle > 30) angle = 30;
    } else {
        // Base, Shoulder, Elbow: Standard 10 -> 170 degrees
        if (angle < 10) angle = 10;
        if (angle > 170) angle = 170;
    }
    
    // Standard pulse mapping: 0 deg = 544us, 180 deg = 2400us
    uint16_t pulseUS = 544 + ((uint32_t)angle * 1856) / 180;
    
    cli(); // Protect atomic write to 16-bit array
    servoPulses[index] = pulseUS * 2; // Multiply by 2 (timer ticks are 0.5us)
    sei();
}

// =============================================================
// Packet helpers
// =============================================================

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine (Pin 21 / PD0 / INT0)
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

volatile unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

ISR(INT0_vect) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastDebounceTime > DEBOUNCE_DELAY) {
        bool isHigh = (PIND & (1 << PD0)); 
        
        if (buttonState == STATE_RUNNING && !isHigh) {
            buttonState = STATE_STOPPED;
            stateChanged = true;
        } 
        else if (buttonState == STATE_STOPPED && isHigh) {
            buttonState = STATE_RUNNING;
            stateChanged = true;
        }
        
        lastDebounceTime = currentMillis;
    }
}

// =============================================================
// Color sensor (TCS3200) - Pins 22-26
// =============================================================

#define S0_BIT PA0 
#define S1_BIT PA1 
#define S2_BIT PA2 
#define S3_BIT PA3 
#define OUT_BIT PA4 

static void colourSensorInit() {
    DDRA |= (1 << S0_BIT) | (1 << S1_BIT) | (1 << S2_BIT) | (1 << S3_BIT);
    DDRA &= ~(1 << OUT_BIT);
    PORTA |= (1 << S0_BIT);
    PORTA &= ~(1 << S1_BIT);
}

static inline void setColorChannel(uint8_t s2, uint8_t s3) {
    if (s2) PORTA |= (1 << S2_BIT); else PORTA &= ~(1 << S2_BIT);
    if (s3) PORTA |= (1 << S3_BIT); else PORTA &= ~(1 << S3_BIT);
}

static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    setColorChannel(s2, s3);
    uint32_t startTime = micros();
    while((uint32_t)micros() - startTime < 5000UL) {} 

    uint32_t count = 0;
    uint8_t last = (PINA & (1 << OUT_BIT)) ? 1 : 0; 
    startTime = micros();
    
    while((uint32_t)micros() - startTime < 100000UL) {
        uint8_t now = (PINA & (1 << OUT_BIT)) ? 1 : 0;
        if (last == 0 && now == 1) count++;
        last = now;
    }
    return count * 10UL; 
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannel(0, 0); 
    *g = measureChannel(1, 1); 
    *b = measureChannel(0, 1); 
}

// =============================================================
// Command handler (Movement & Arm Integrated)
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            stop(); 
            sendResponse(RESP_OK, 0);
            sendStatus(STATE_STOPPED);
            break;

        case COMMAND_COLOR:
        {
            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            pkt.params[0]  = r;
            pkt.params[1]  = g;
            pkt.params[2]  = b;
            sendFrame(&pkt);
            break;
        }

        // --- Movement Commands (Hardcoded to Max Speed) ---
        case COMMAND_FORWARD:
            forward(robotSpeed);
            lastMoveTime = millis();
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_BACKWARD:
            backward(robotSpeed);
            lastMoveTime = millis();
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_TURN_LEFT:
            ccw(robotSpeed);
            lastMoveTime = millis();
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_TURN_RIGHT:
            cw(robotSpeed);
            lastMoveTime = millis();
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_STOP:
            stop();
            sendResponse(RESP_OK, 0);
            break;
            
        // Ignore math for speed changes, just return the constant 255 to the Pi
        case COMMAND_SPEED_UP:
            sendResponse(RESP_OK, robotSpeed);
            break;
        case COMMAND_SPEED_DOWN:
            sendResponse(RESP_OK, robotSpeed);
            break;

        // --- 4-DOF Bare-Metal Arm Commands ---
        case COMMAND_GRIPPER_OPEN:
            gripperAngle -= 5;
            setServoAngle(3, gripperAngle);
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_GRIPPER_CLOSE:
            gripperAngle += 5;
            setServoAngle(3, gripperAngle);
            sendResponse(RESP_OK, 0);
            break;
            
        case COMMAND_SET_BASE:
            baseAngle = cmd->params[0]; // Grab user angle from packet
            setServoAngle(0, baseAngle);
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_SET_SHOULDER:
            shoulderAngle = cmd->params[0]; // Grab user angle from packet
            setServoAngle(1, shoulderAngle);
            sendResponse(RESP_OK, 0);
            break;
        case COMMAND_SET_ELBOW:
            elbowAngle = cmd->params[0]; // Grab user angle from packet
            setServoAngle(2, elbowAngle);
            sendResponse(RESP_OK, 0);
            break;
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    // 1. Configure E-Stop Pin (Pin 21 / PD0)
    DDRD &= ~(1 << PD0); 
    PORTD |= (1 << PD0); 
    
    // 2. Configure INT0 to fire on ANY logic change
    EICRA |= (1 << ISC00);
    EICRA &= ~(1 << ISC01);
    
    // 3. Enable INT0 in the mask register
    EIMSK |= (1 << INT0);

    // 4. Initialize Color Sensor
    colourSensorInit();

    // 5. Initialize Bare-Metal Servos
    bareMetalServoInit();
    setServoAngle(0, baseAngle);     // Index 0: Base
    setServoAngle(1, shoulderAngle); // Index 1: Shoulder
    setServoAngle(2, elbowAngle);    // Index 2: Elbow
    setServoAngle(3, gripperAngle);  // Index 3: Gripper (Initializes to 15)

    // 6. Ensure all motors start stopped
    stop();

    sei();
}

void loop() {
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        
        if (state == STATE_STOPPED) {
            stop(); 
        }
        sendStatus(state);
    }

    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }

    // --- The Heartbeat Timeout ---
    if (millis() - lastMoveTime > MOVE_TIMEOUT) {
        stop();
    }
}