/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

volatile unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */
ISR(INT0_vect) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastDebounceTime > DEBOUNCE_DELAY) {
        // Read the bare-metal state of Pin 21 (PD0)
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
// Color sensor (TCS3200)
// =============================================================

#define OUT_PIN 2
#define S0 3
#define S1 4
#define S2 5
#define S3 6

static void colourSensorInit() {
    // Configure S pins as OUTPUT and OUT_PIN as INPUT
    DDRA |= (1 << S0) | (1 << S1) | (1 << S2) | (1 << S3);
    DDRA &= ~(1 << OUT_PIN);
    
    // Set to 20% frequency scaling [cite: 589]
    PORTA |= (1 << S0);
    PORTA &= ~(1 << S1);
}

static inline void setColorChannel(uint8_t s2, uint8_t s3) {
    if (s2) PORTA |= (1 << S2);
    else    PORTA &= ~(1 << S2);
    
    if (s3) PORTA |= (1 << S3);
    else    PORTA &= ~(1 << S3);
}

static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    setColorChannel(s2, s3);
    uint32_t startTime = micros();
    // Allow sensor to settle after changing filters
    while((uint32_t)micros() - startTime < 5000UL) {} 

    uint32_t count = 0;
    uint8_t last = (PINA & (1 << OUT_PIN)) ? 1 : 0;
    startTime = micros();
    
    // Count rising edges over a 100ms window [cite: 615]
    while((uint32_t)micros() - startTime < 100000UL) {
        uint8_t now = (PINA & (1 << OUT_PIN)) ? 1 : 0;
        if (last == 0 && now == 1) count++;
        last = now;
    }
    return count * 10UL; // Convert to Hz [cite: 615]
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannel(0, 0); // Red
    *g = measureChannel(1, 1); // Green
    *b = measureChannel(0, 1); // Blue
}


// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
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
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    DDRD &= ~(1 << PD0);
    EICRA |= (1 << ISC00);
    EICRA &= ~(1 << ISC01);
    EIMSK |= (1 << INT0);

    colourSensorInit();

    sei();
}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
