/*
 * servo_calibrate.ino
 * Stripped-down Bare-Metal Servo calibrator (BREADBOARD EDITION).
 * Pins: Base(45), Shoulder(47), Elbow(49), Gripper(51)
 */

volatile uint16_t servoPulses[4] = {3000, 3000, 3000, 1054}; // Defaults: 90, 90, 90, 15

void bareMetalServoInit() {
    // 1. Set NEW pins as outputs
    DDRL |= (1 << PL4);  // Pin 45 (Base)
    DDRL |= (1 << PL2);  // Pin 47 (Shoulder)
    DDRL |= (1 << PL0);  // Pin 49 (Elbow)
    DDRB |= (1 << PB2);  // Pin 51 (Gripper)

    cli();
    TCCR5A = 0; TCCR5B = 0; TCNT5 = 0;
    TCCR5B |= (1 << WGM52); 
    TCCR5B |= (1 << CS51);  
    OCR5A = 1000;           
    TIMSK5 |= (1 << OCIE5A); 
    sei();
}

ISR(TIMER5_COMPA_vect) {
    static uint8_t servoIndex = 0;
    static uint16_t totalTicks = 0;

    // Turn OFF the previous servo pin
    if (servoIndex == 1)      PORTL &= ~(1 << PL4); // Base OFF
    else if (servoIndex == 2) PORTL &= ~(1 << PL2); // Shoulder OFF
    else if (servoIndex == 3) PORTL &= ~(1 << PL0); // Elbow OFF
    else if (servoIndex == 4) PORTB &= ~(1 << PB2); // Gripper OFF

    // Turn ON the current servo pin and set its duration
    if (servoIndex < 4) {
        if (servoIndex == 0)      PORTL |= (1 << PL4); // Base ON
        else if (servoIndex == 1) PORTL |= (1 << PL2); // Shoulder ON
        else if (servoIndex == 2) PORTL |= (1 << PL0); // Elbow ON
        else if (servoIndex == 3) PORTB |= (1 << PB2); // Gripper ON

        OCR5A = servoPulses[servoIndex];
        totalTicks += servoPulses[servoIndex];
        servoIndex++;
    } else {
        if (totalTicks < 40000) OCR5A = 40000 - totalTicks;
        else OCR5A = 1000;
        servoIndex = 0;
        totalTicks = 0;
    }
}

void setup() {
    Serial.begin(9600);
    bareMetalServoInit();
    Serial.println("Calibration Firmware Ready. Waiting for commands...");
}

void loop() {
    // Listen for simple string commands formatted as "Index,Angle\n"
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        int commaIdx = cmd.indexOf(',');
        
        if (commaIdx > 0) {
            int idx = cmd.substring(0, commaIdx).toInt();
            int ang = cmd.substring(commaIdx + 1).toInt();
            
            // Absolute hardware limits of standard servos
            if (ang < 0) ang = 0;
            if (ang > 180) ang = 180;
            
            uint16_t pulseUS = 544 + ((uint32_t)ang * 1856) / 180;
            
            cli();
            servoPulses[idx] = pulseUS * 2;
            sei();
            
            Serial.print("Servo "); Serial.print(idx);
            Serial.print(" moved to "); Serial.println(ang);
        }
    }
}