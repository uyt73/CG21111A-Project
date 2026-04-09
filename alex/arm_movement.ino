#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <ctype.h>

#define BASE_BIT      0   // PL0 -> pin 49
#define GRIPPER_BIT   2   // PB2 -> pin 51
#define SHOULDER_BIT  6   // PH6 -> pin 9
#define ELBOW_BIT     4   // PB4 -> pin 10

volatile uint16_t pulse[4] = {1500, 1500, 1500, 1500};
int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 0;
int msPerDeg = 10;

int parse3(const String *s) {
  if (!s || s->length() != 3) return -1;
  for (int i = 0; i < 3; i++) {
    if (!isDigit(s->charAt(i))) return -1;
  }
  return (s->charAt(0) - '0') * 100 +
         (s->charAt(1) - '0') * 10 +
         (s->charAt(2) - '0');
}

uint16_t degToPulse(int d) {
  d = constrain(d, 0, 180);
  return 500 + (uint32_t)d * 2000 / 180;
}

static inline void setPulseAtomic(uint8_t idx, uint16_t us) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pulse[idx] = us;
  }
}

void moveSmooth(uint8_t idx, int *cur, int target) {
  target = constrain(target, 0, 180);
  if (idx == 3) target = constrain(target, 0, 30);  // gripper limit

  if (*cur == target) return;
  int step = (target > *cur) ? 1 : -1;
  while (*cur != target) {
    *cur += step;
    setPulseAtomic(idx, degToPulse(*cur));
    delay(msPerDeg);
  }
}

void homeAll() {
  moveSmooth(0, &basePos, 90);
  moveSmooth(1, &shoulderPos, 90);
  moveSmooth(2, &elbowPos, 90);
  moveSmooth(3, &gripperPos, 0);
}

// Timer1 @ prescaler 8 => 2 MHz => 0.5 us / tick
// Servo pulses are output sequentially in one 20 ms frame.
ISR(TIMER1_COMPA_vect) {
  static uint8_t phase = 0;     // 0..4
  static uint16_t used_us = 0;  // total pulse widths used this frame

  uint16_t p[4];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    p[0] = pulse[0];
    p[1] = pulse[1];
    p[2] = pulse[2];
    p[3] = pulse[3];
  }

  switch (phase) {
    case 0: // BASE high
      used_us = 0;
      PORTL |= (1 << BASE_BIT);        // D49 high
      OCR1A = p[0] * 2;
      used_us += p[0];
      phase = 1;
      break;

    case 1: // BASE low, SHOULDER high
      PORTL &= ~(1 << BASE_BIT);       // D49 low
      PORTH |= (1 << SHOULDER_BIT);    // D9 high
      OCR1A = p[1] * 2;
      used_us += p[1];
      phase = 2;
      break;

    case 2: // SHOULDER low, ELBOW high
      PORTH &= ~(1 << SHOULDER_BIT);   // D9 low
      PORTB |= (1 << ELBOW_BIT);       // D10 high
      OCR1A = p[2] * 2;
      used_us += p[2];
      phase = 3;
      break;

    case 3: // ELBOW low, GRIPPER high
      PORTB &= ~(1 << ELBOW_BIT);      // D10 low
      PORTB |= (1 << GRIPPER_BIT);     // D51 high
      OCR1A = p[3] * 2;
      used_us += p[3];
      phase = 4;
      break;

    case 4: { // GRIPPER low, wait remainder of 20 ms
      PORTB &= ~(1 << GRIPPER_BIT);    // D51 low
      uint16_t rem_us = (used_us < 20000) ? (20000 - used_us) : 1000;
      OCR1A = rem_us * 2;
      phase = 0;
      break;
    }
  }
}

void setup() {
  Serial.begin(115200); // Note the higher baud rate!

  // Set output pins
  DDRL |= (1 << BASE_BIT);        // D49
  DDRH |= (1 << SHOULDER_BIT);    // D9
  DDRB |= (1 << ELBOW_BIT);       // D10
  DDRB |= (1 << GRIPPER_BIT);     // D51

  // Set all low initially
  PORTL &= ~(1 << BASE_BIT);
  PORTH &= ~(1 << SHOULDER_BIT);
  PORTB &= ~(1 << ELBOW_BIT);
  PORTB &= ~(1 << GRIPPER_BIT);

  // Initialize pulses from starting angles
  setPulseAtomic(0, degToPulse(basePos));
  setPulseAtomic(1, degToPulse(shoulderPos));
  setPulseAtomic(2, degToPulse(elbowPos));
  setPulseAtomic(3, degToPulse(gripperPos));

  // Timer1 CTC, prescaler /8
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 40000; // 20 ms initial delay
  TIMSK1 = (1 << OCIE1A);

  sei();
}

void loop() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;

  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  char c = cmd.charAt(0);
  String arg = cmd.substring(1);
  int val = parse3(&arg);
  
  if (val < 0) {
    Serial.println("ERROR: Argument not valid");
    return;
  }

  if (c == 'V') {
    msPerDeg = val;
    Serial.print("Speed set to ");
    Serial.println(msPerDeg);
  } else if (c == 'B') {
    moveSmooth(0, &basePos, val);
  } else if (c == 'S') {
    moveSmooth(1, &shoulderPos, val);
  } else if (c == 'E') {
    moveSmooth(2, &elbowPos, val);
  } else if (c == 'G') {
    moveSmooth(3, &gripperPos, val);
  } else {
    Serial.println("ERROR: Unknown command");
  }
}