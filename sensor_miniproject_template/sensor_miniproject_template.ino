/*
 * sensor_miniproject_template.ino
 * Studio 15 Merge: Movement & Sensors
 */

#include "packets.h"
#include "serial_driver.h"

// Speed variable for movement commands [cite: 299]
int robotSpeed = 150; 

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
// E-Stop state machine (Pin 21 / PD0 / INT0) [cite: 258]
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
// Color sensor (TCS3200) - Pins 22-26 [cite: 264]
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
// Command handler (Movement commands integrated) [cite: 279]
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            stop(); // Ensure motors stop on software E-stop [cite: 306]
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

        // --- Studio 15 Movement Commands --- [cite: 217, 305]
        case COMMAND_FORWARD:
            forward(robotSpeed);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_BACKWARD:
            backward(robotSpeed);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_TURN_LEFT:
            ccw(robotSpeed);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_TURN_RIGHT:
            cw(robotSpeed);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_STOP:
            stop();
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_SPEED_UP:
            robotSpeed += 20;
            if (robotSpeed > 255) robotSpeed = 255;
            sendResponse(RESP_OK, robotSpeed);
            break;

        case COMMAND_SPEED_DOWN:
            robotSpeed -= 20;
            if (robotSpeed < 0) robotSpeed = 0;
            sendResponse(RESP_OK, robotSpeed);
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

    // 1. Configure E-Stop Pin (Pin 21 / PD0) [cite: 289]
    DDRD &= ~(1 << PD0); 
    PORTD |= (1 << PD0); 
    
    // 2. Configure INT0 to fire on ANY logic change [cite: 291]
    EICRA |= (1 << ISC00);
    EICRA &= ~(1 << ISC01);
    
    // 3. Enable INT0 in the mask register [cite: 292]
    EIMSK |= (1 << INT0);

    // 4. Initialize Color Sensor [cite: 293]
    colourSensorInit();

    // 5. Ensure all motors start stopped
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
            stop(); // Cut motor power immediately if button pressed [cite: 306]
        }
        sendStatus(state);
    }

    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}