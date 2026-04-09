#include "packets.h"
#include "serial_driver.h"

// =============================================================
// STATE VARIABLES
// =============================================================
int currentSpeed = 150; 

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

volatile unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// =============================================================
// 1. PHYSICAL E-STOP BUTTON (INT0 on Pin 21 / PD0)
// =============================================================
ISR(INT0_vect) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastDebounceTime > DEBOUNCE_DELAY) {
        bool isHigh = (PIND & (1 << PD0)); 
        
        if (buttonState == STATE_RUNNING && isHigh) {
            buttonState = STATE_STOPPED;
            stateChanged = true;
        } 
        else if (buttonState == STATE_STOPPED && !isHigh) {
            buttonState = STATE_RUNNING;
            stateChanged = true;
        }
        
        lastDebounceTime = currentMillis;
    }
}

// =============================================================
// 2. COLOR SENSOR - BAREMETAL ATMEGA2560 (Pins 22-26)
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
// 3. MOTOR WRAPPERS (Calls functions from robotlib.ino)
// =============================================================
void driveForward()  { forward(currentSpeed); }
void driveBackward() { backward(currentSpeed); }
void turnLeft()      { ccw(currentSpeed); }
void turnRight()     { cw(currentSpeed); }

void changeSpeed(int delta) {
    currentSpeed += delta;
    if (currentSpeed > 255) currentSpeed = 255;
    if (currentSpeed < 0) currentSpeed = 0;
}

// =============================================================
// 4. PACKET COMMUNICATION HELPERS
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

void clearEstop() {
    buttonState = STATE_RUNNING;
    sendStatus(STATE_RUNNING);
}

// =============================================================
// 5. THE COMMAND HANDLER (The Brains)
// =============================================================
void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_FORWARD:    driveForward();  break;
        case COMMAND_BACKWARD:   driveBackward(); break;
        case COMMAND_TURN_LEFT:  turnLeft();      break;
        case COMMAND_TURN_RIGHT: turnRight();     break;
        case COMMAND_SPEED_UP:   changeSpeed(25); break;
        case COMMAND_SPEED_DOWN: changeSpeed(-25);break;
        
        case COMMAND_STOP:       
            stop();          
            break;
            
        case COMMAND_ESTOP:
            cli();
            buttonState = STATE_STOPPED;
            stateChanged = false;
            sei();
            stop(); // Physically halt the robot!
            sendStatus(STATE_STOPPED);
            break;
            
        case COMMAND_CLEAR_ESTOP: 
            clearEstop();    
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
    }
}

// =============================================================
// 6. SETUP & LOOP
// =============================================================
void setup() {
#if USE_BAREMETAL_SERIAL
    usartInit(103); 
#else
    Serial.begin(9600);
#endif

    // Init E-Stop Button Interrupt (INT0)
    DDRD &= ~(1 << PD0);
    EICRA |= (1 << ISC00);
    EICRA &= ~(1 << ISC01);
    EIMSK |= (1 << INT0);

    // Init Color Sensor Pins
    colourSensorInit();

    sei(); 
}

void loop() {
    // Check if the physical button was pressed/released
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        
        // If the button was pressed, kill the motors instantly
        if (state == STATE_STOPPED) {
            stop(); 
        }
        // Tell the Pi the state changed
        sendStatus(state);
    }

    // Check for incoming network packets
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}