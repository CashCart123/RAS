#include <Servo.h>

// -----------------------------
// USER CONFIGURATION
// -----------------------------
#define SERVO_PIN A0        // Change if needed
#define LED_PIN   13       // Built-in LED usually 13

// Servo angles
#define RAISED_ANGLE 90
#define LOWERED_ANGLE 0

// Set to true if the servo rotates the wrong way
#define INVERT_DIRECTION false
// -----------------------------

Servo mastServo;

String inputString = "";
bool mastRaised = false;

void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  mastServo.attach(SERVO_PIN);

  // Start in lowered position
  moveLower();
}

void loop() {

  if (Serial.available()) {

    inputString = Serial.readStringUntil('\n');
    inputString.trim();
    inputString.toLowerCase();

    if (inputString == "raise") {
      moveRaise();
    }

    if (inputString == "lower") {
      moveLower();
    }
  }
}

void moveRaise() {

  int angle = RAISED_ANGLE;

  if (INVERT_DIRECTION)
    angle = 180 - RAISED_ANGLE;

  mastServo.write(angle);

  digitalWrite(LED_PIN, HIGH);

  mastRaised = true;

  Serial.println("MAST RAISED");
}

void moveLower() {

  int angle = LOWERED_ANGLE;

  if (INVERT_DIRECTION)
    angle = 180 - LOWERED_ANGLE;

  mastServo.write(angle);

  digitalWrite(LED_PIN, LOW);

  mastRaised = false;

  Serial.println("MAST LOWERED");
}