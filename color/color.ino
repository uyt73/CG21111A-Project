// TCS3200 Color Sensor Standalone Diagnostic
// Pins mapped for Arduino Mega

#define S0 22
#define S1 23
#define S2 24
#define S3 25
#define OUT 26

void setup() {
  Serial.begin(9600);
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  
  // Set to 20% frequency scaling (Standard for GY-31)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  Serial.println("TCS3200 Diagnostic Booted!");
  Serial.println("Waiting for sensor readings...");
  delay(1000);
}

void loop() {
  int redFrequency = 0;
  int greenFrequency = 0;
  int blueFrequency = 0;

  // 1. Read RED Channel (S2=LOW, S3=LOW)
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(50); // Give the sensor a moment to settle
  redFrequency = pulseIn(OUT, LOW, 100000); // 100ms timeout

  // 2. Read GREEN Channel (S2=HIGH, S3=HIGH)
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(50);
  greenFrequency = pulseIn(OUT, LOW, 100000);

  // 3. Read BLUE Channel (S2=LOW, S3=HIGH)
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(50);
  blueFrequency = pulseIn(OUT, LOW, 100000);

  // Print results to the Serial Monitor
  Serial.print("R: ");
  Serial.print(redFrequency);
  Serial.print("  |  G: ");
  Serial.print(greenFrequency);
  Serial.print("  |  B: ");
  Serial.println(blueFrequency);

  delay(500); // Read twice a second
}