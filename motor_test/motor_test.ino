// Standalone Motor Shield Diagnostic Test
// Requires the Adafruit Motor Shield V1 library (<AFMotor.h>)

#include <AFMotor.h>

// Initialize all 4 motor ports on the shield
AF_DCMotor motor1(1); // M1
AF_DCMotor motor2(2); // M2
AF_DCMotor motor3(3); // M3
AF_DCMotor motor4(4); // M4

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Shield Diagnostic Booted!");

  // Set speed to 200 (out of 255) - high enough to overcome friction
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  
  // Ensure all motors are off to start
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  
  delay(2000);
}

void loop() {
  Serial.println("Testing Motors: FORWARD");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(3000); // Run for 3 seconds

  Serial.println("Testing Motors: STOP");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  delay(2000); // Pause for 2 seconds

  Serial.println("Testing Motors: BACKWARD");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(3000); // Run for 3 seconds

  Serial.println("Testing Motors: STOP");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  delay(2000); // Pause for 2 seconds
}