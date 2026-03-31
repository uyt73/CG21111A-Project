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

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

// #define ESTOP_DDR   DDRE
// #define ESTOP_PORT  PORTE
// #define ESTOP_PINR  PINE
// #define ESTOP_BIT   4       // PE4 = Arduino Mega D2 = INT4

#define ESTOP_DDR   DDRK
#define ESTOP_PORT  PORTK
#define ESTOP_PINR  PINK
#define ESTOP_BIT PCINT23 // A15 Arduino Mega

volatile unsigned long lastDebounceTime = 0;
#define DEBOUNCE_MS 250

ISR(PCINT2_vect) {
    unsigned long now = millis();

    if ((now - lastDebounceTime) < DEBOUNCE_MS) return;
    lastDebounceTime = now;

    uint8_t pinHigh = (ESTOP_PINR & (1 << ESTOP_BIT)) ? 1 : 0;

    if (buttonState == STATE_RUNNING && pinHigh) {
        buttonState = STATE_STOPPED;
        stateChanged = true;
    }
    else if (buttonState == STATE_STOPPED && !pinHigh) {
        buttonState = STATE_RUNNING;
        stateChanged = true;
    }
}


// =============================================================
// Color sensor (TCS3200)
// =============================================================

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */


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
 
//PORTA

#define OUT_PIN 2
#define S0 3
#define S1 4
#define S2 5
#define S3 6

static void colourSensorInit() {
    // Configure S pins AND OUTpin
    DDRA |= (1 << S0) | (1 << S1) | (1 << S2) | (1 << S3);
    DDRA &= ~(1 << OUT_PIN);
    // Setting to 20%
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
    while((uint32_t)micros() - startTime < 5000UL) {}

    uint32_t count = 0;
    uint8_t last = (PINA & (1 << OUT_PIN)) ? 1 : 0;
    startTime = micros();
    while((uint32_t)micros() - startTime < 100000UL) {
        uint8_t now = (PINA & (1 << OUT_PIN)) ? 1 : 0;
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

int currentSpeed = 150; // Default starting speed

void driveForward() {
    // TODO: Call your robotlib.ino forward function here
    forward(currentSpeed);
}
void driveBackward() {
    // TODO: Call your robotlib.ino backward function here
    backward(currentSpeed);
}
void turnLeft() {
    // TODO: Call your robotlib.ino left function here
    ccw(currentSpeed);   
}
void turnRight() {
    // TODO: Call your robotlib.ino right function here
    cw(currentSpeed);
}
void speedUp() {
    currentSpeed += 25;
    if (currentSpeed > 255) currentSpeed = 255;
    // TODO: Apply currentSpeed to your motors here
}
void speedDown() {
    currentSpeed -= 25;
    if (currentSpeed < 0) currentSpeed = 0;
    // TODO: Apply currentSpeed to your motors here
}


static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            stop();
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

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
        case COMMAND_COLOR:
        {
            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);
            TPacket pkt; 
             memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command = RESP_COLOR;
            pkt.params[0] = r;
            pkt.params[1] = g;
            pkt.params[2] = b;
            sendFrame(&pkt);
            break;
        }

        case COMMAND_FORWARD:
            driveForward();
            break;
        case COMMAND_BACKWARD:
            driveBackward();
            break;
        case COMMAND_TURN_LEFT:
            turnLeft();
            break;
        case COMMAND_TURN_RIGHT:
            turnRight();
            break;
        case COMMAND_SPEED_UP:
            speedUp();
            break;
        case COMMAND_SPEED_DOWN:
            speedDown();
            break;

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
    // then call sei() to enable global interrupts.
    ESTOP_DDR &= ~(1 << ESTOP_BIT);
    ESTOP_PORT &= ~(1 << ESTOP_BIT);

    PCICR |= (1 << PCIE1);
    PCMSK |= (1 << ESTOP_BIT);

    // EICRB &= ~(1 << ISC41);
    // EICRB |=  (1 << ISC40);

    // EIMSK |= (1 << INT4);

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
