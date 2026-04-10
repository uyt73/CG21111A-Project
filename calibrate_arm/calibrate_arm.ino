/*
 * servo_calibrate.ino
 * Stripped-down Bare-Metal Servo calibrator.
 */

volatile uint16_t servoPulses[4] = {3000, 3000, 3000, 1054}; // Defaults: 90, 90, 90, 15

void bareMetalServoInit() {
    DDRL |= (1 << PL0);  // Pin 49 (Base)
    DDRH |= (1 << PH6);  // Pin 9  (Shoulder)
    DDRB |= (1 << PB4);  // Pin 10 (Elbow)
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

    if (servoIndex == 1)      PORTL &= ~(1 << PL0);
    else if (servoIndex == 2) PORTH &= ~(1 << PH6);
    else if (servoIndex == 3) PORTB &= ~(1 << PB4);
    else if (servoIndex == 4) PORTB &= ~(1 << PB2);

    if (servoIndex < 4) {
        if (servoIndex == 0)      PORTL |= (1 << PL0);
        else if (servoIndex == 1) PORTH |= (1 << PH6);
        else if (servoIndex == 2) PORTB |= (1 << PB4);
        else if (servoIndex == 3) PORTB |= (1 << PB2);

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