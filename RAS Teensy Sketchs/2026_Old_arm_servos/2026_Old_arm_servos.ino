#include <Servo.h>

// -----------------------------
// USER CONFIGURATION
// -----------------------------
#define SERVO1_PIN A0  // poke / retract
#define SERVO2_PIN A1  // spin_left / spin_right
#define LED_PIN 13

#define SERIAL_BAUD 115200
#define CMD_BUFFER_SIZE 32

// Servo 1 angles (poke mechanism)
#define POKE_ANGLE 90
#define RETRACT_ANGLE 10

// Servo 2 angles (spin mechanism)
#define SPIN_LEFT_ANGLE 20
#define SPIN_RIGHT_ANGLE 90

// Set individually if either servo moves the wrong way
#define INVERT_SERVO1 false
#define INVERT_SERVO2 false
// -----------------------------

Servo servo1;
Servo servo2;

enum Servo1State {
  SERVO1_UNKNOWN = 0,
  SERVO1_POKED,
  SERVO1_RETRACTED
};

enum Servo2State {
  SERVO2_UNKNOWN = 0,
  SERVO2_LEFT,
  SERVO2_RIGHT
};

Servo1State servo1State = SERVO1_UNKNOWN;
Servo2State servo2State = SERVO2_UNKNOWN;

// Serial command buffer
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// -----------------------------
// FUNCTION DECLARATIONS
// -----------------------------
void processSerial();
void handleCommand(char *cmd);
void normalizeCommand(char *cmd);
void clearCommandBuffer();

void movePoke();
void moveRetract();
void moveSpinLeft();
void moveSpinRight();

void moveServo1ToState(Servo1State targetState);
void moveServo2ToState(Servo2State targetState);

int getServo1AngleForState(Servo1State state);
int getServo2AngleForState(Servo2State state);

const char *servo1StateToString(Servo1State state);
const char *servo2StateToString(Servo2State state);

void printStatus();

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  clearCommandBuffer();

  Serial.println("DUAL SERVO CONTROLLER STARTING");
  Serial.println("Available commands: poke, retract, spin_left, spin_right, status");

  // Safe startup positions
  moveRetract();
  moveSpinRight();

  printStatus();
}

// -----------------------------
// MAIN LOOP
// -----------------------------
void loop() {
  processSerial();
}

// -----------------------------
// SERIAL PROCESSING
// -----------------------------
void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        handleCommand(cmdBuffer);
        clearCommandBuffer();
      }
      continue;
    }

    if (c >= 32 && c <= 126) {
      if (cmdIndex < (CMD_BUFFER_SIZE - 1)) {
        cmdBuffer[cmdIndex++] = c;
      } else {
        Serial.println("ERROR: command too long, discarded");
        clearCommandBuffer();

        while (Serial.available() > 0) {
          char flushChar = Serial.read();
          if (flushChar == '\n' || flushChar == '\r') {
            break;
          }
        }
      }
    }
  }
}

// -----------------------------
// COMMAND HANDLING
// -----------------------------
void handleCommand(char *cmd) {
  normalizeCommand(cmd);

  if (strlen(cmd) == 0) {
    return;
  }

  Serial.print("CMD: ");
  Serial.println(cmd);

  if (strcmp(cmd, "poke") == 0) {
    movePoke();
  } else if (strcmp(cmd, "retract") == 0) {
    moveRetract();
  } else if (strcmp(cmd, "spin_left") == 0) {
    moveSpinLeft();
  } else if (strcmp(cmd, "spin_right") == 0) {
    moveSpinRight();
  } else if (strcmp(cmd, "status") == 0) {
    printStatus();
  } else {
    Serial.print("ERROR: unknown command: ");
    Serial.println(cmd);
    Serial.println("Valid commands: poke, retract, spin_left, spin_right, status");
  }
}

// Converts to lowercase and trims leading/trailing whitespace
void normalizeCommand(char *cmd) {
  char *start = cmd;
  while (*start == ' ' || *start == '\t') {
    start++;
  }

  if (start != cmd) {
    memmove(cmd, start, strlen(start) + 1);
  }

  int len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) {
    cmd[len - 1] = '\0';
    len--;
  }

  for (int i = 0; cmd[i] != '\0'; i++) {
    if (cmd[i] >= 'A' && cmd[i] <= 'Z') {
      cmd[i] = cmd[i] - 'A' + 'a';
    }
  }
}

void clearCommandBuffer() {
  memset(cmdBuffer, 0, sizeof(cmdBuffer));
  cmdIndex = 0;
}

// -----------------------------
// MOVEMENT LOGIC
// -----------------------------
void movePoke() {
  moveServo1ToState(SERVO1_POKED);
}

void moveRetract() {
  moveServo1ToState(SERVO1_RETRACTED);
}

void moveSpinLeft() {
  moveServo2ToState(SERVO2_LEFT);
}

void moveSpinRight() {
  moveServo2ToState(SERVO2_RIGHT);
}

void moveServo1ToState(Servo1State targetState) {
  if (targetState == servo1State) {
    Serial.print("OK: servo1 already ");
    Serial.println(servo1StateToString(servo1State));
    return;
  }

  int angle = getServo1AngleForState(targetState);
  servo1.write(angle);
  servo1State = targetState;

  Serial.print("OK: servo1 ");
  Serial.print(servo1StateToString(servo1State));
  Serial.print(" (angle ");
  Serial.print(angle);
  Serial.println(")");
}

void moveServo2ToState(Servo2State targetState) {
  if (targetState == servo2State) {
    Serial.print("OK: servo2 already ");
    Serial.println(servo2StateToString(servo2State));
    return;
  }

  int angle = getServo2AngleForState(targetState);
  servo2.write(angle);
  servo2State = targetState;

  Serial.print("OK: servo2 ");
  Serial.print(servo2StateToString(servo2State));
  Serial.print(" (angle ");
  Serial.print(angle);
  Serial.println(")");
}

int getServo1AngleForState(Servo1State state) {
  int angle = RETRACT_ANGLE;

  switch (state) {
    case SERVO1_POKED:
      angle = POKE_ANGLE;
      break;
    case SERVO1_RETRACTED:
      angle = RETRACT_ANGLE;
      break;
    default:
      angle = RETRACT_ANGLE;
      break;
  }

  angle = constrain(angle, 0, 180);

  if (INVERT_SERVO1) {
    angle = 180 - angle;
  }

  return constrain(angle, 0, 180);
}

int getServo2AngleForState(Servo2State state) {
  int angle = SPIN_RIGHT_ANGLE;

  switch (state) {
    case SERVO2_LEFT:
      angle = SPIN_LEFT_ANGLE;
      break;
    case SERVO2_RIGHT:
      angle = SPIN_RIGHT_ANGLE;
      break;
    default:
      angle = SPIN_RIGHT_ANGLE;
      break;
  }

  angle = constrain(angle, 0, 180);

  if (INVERT_SERVO2) {
    angle = 180 - angle;
  }

  return constrain(angle, 0, 180);
}

const char *servo1StateToString(Servo1State state) {
  switch (state) {
    case SERVO1_POKED: return "POKED";
    case SERVO1_RETRACTED: return "RETRACTED";
    default: return "UNKNOWN";
  }
}

const char *servo2StateToString(Servo2State state) {
  switch (state) {
    case SERVO2_LEFT: return "LEFT";
    case SERVO2_RIGHT: return "RIGHT";
    default: return "UNKNOWN";
  }
}

void printStatus() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // blink LED on each status call

  Serial.print("STATUS: servo1=");
  Serial.print(servo1StateToString(servo1State));
  Serial.print(" (angle=");
  Serial.print(getServo1AngleForState(servo1State));
  Serial.print("), servo2=");
  Serial.print(servo2StateToString(servo2State));
  Serial.print(" (angle=");
  Serial.print(getServo2AngleForState(servo2State));
  Serial.println(")");
}