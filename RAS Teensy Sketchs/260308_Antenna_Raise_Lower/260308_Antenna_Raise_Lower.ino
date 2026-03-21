#include <Servo.h>

// -----------------------------
// USER CONFIGURATION
// -----------------------------
#define SERVO_PIN A0
#define LED_PIN   13

#define RAISED_ANGLE  100
#define LOWERED_ANGLE 0
#define STOWED_ANGLE  30

#define INVERT_DIRECTION false

// Serial settings
#define SERIAL_BAUD 115200

// Max command length (excluding null terminator)
#define CMD_BUFFER_SIZE 32
// -----------------------------

Servo mastServo;

enum MastState {
  MAST_UNKNOWN = 0,
  MAST_RAISED,
  MAST_LOWERED,
  MAST_STOWED
};

MastState mastState = MAST_UNKNOWN;

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
void moveRaise();
void moveLower();
void moveStow();
void moveToState(MastState targetState);
void printStatus();
int getServoAngleForState(MastState state);
const char* stateToString(MastState state);

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  mastServo.attach(SERVO_PIN);

  clearCommandBuffer();

  Serial.println("MAST CONTROLLER STARTING");
  Serial.println("Available commands: raise, lower, stow, status");

  // Start in lowered position
  moveLower();
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

    // End of command
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        handleCommand(cmdBuffer);
        clearCommandBuffer();
      }
      // Ignore repeated CR/LF characters
      continue;
    }

    // Only store printable characters up to buffer limit
    if (c >= 32 && c <= 126) {
      if (cmdIndex < (CMD_BUFFER_SIZE - 1)) {
        cmdBuffer[cmdIndex++] = c;
      } else {
        Serial.println("ERROR: command too long, discarded");
        clearCommandBuffer();

        // Flush remainder of the line
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

  if (strcmp(cmd, "raise") == 0) {
    moveRaise();
  } else if (strcmp(cmd, "lower") == 0) {
    moveLower();
  } else if (strcmp(cmd, "stow") == 0) {
    moveStow();
  } else if (strcmp(cmd, "status") == 0) {
    printStatus();
  } else {
    Serial.print("ERROR: unknown command: ");
    Serial.println(cmd);
    Serial.println("Valid commands: raise, lower, stow, status");
  }
}

// Converts to lowercase and trims leading/trailing whitespace
void normalizeCommand(char *cmd) {
  // Trim leading spaces
  char *start = cmd;
  while (*start == ' ' || *start == '\t') {
    start++;
  }

  // Move trimmed string to front if needed
  if (start != cmd) {
    memmove(cmd, start, strlen(start) + 1);
  }

  // Trim trailing spaces
  int len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) {
    cmd[len - 1] = '\0';
    len--;
  }

  // Convert to lowercase
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
void moveRaise() {
  moveToState(MAST_RAISED);
}

void moveLower() {
  moveToState(MAST_LOWERED);
}

void moveStow() {
  moveToState(MAST_STOWED);
}

void moveToState(MastState targetState) {
  if (targetState == mastState) {
    Serial.print("OK: already ");
    Serial.println(stateToString(mastState));
    return;
  }

  int angle = getServoAngleForState(targetState);
  mastServo.write(angle);

  // LED behavior: on only when raised
  if (targetState == MAST_RAISED) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  mastState = targetState;

  Serial.print("OK: mast ");
  Serial.print(stateToString(mastState));
  Serial.print(" (servo angle ");
  Serial.print(angle);
  Serial.println(")");
}

int getServoAngleForState(MastState state) {
  int angle = 0;

  switch (state) {
    case MAST_RAISED:
      angle = RAISED_ANGLE;
      break;
    case MAST_LOWERED:
      angle = LOWERED_ANGLE;
      break;
    case MAST_STOWED:
      angle = STOWED_ANGLE;
      break;
    default:
      angle = LOWERED_ANGLE;
      break;
  }

  angle = constrain(angle, 0, 180);

  if (INVERT_DIRECTION) {
    angle = 180 - angle;
  }

  angle = constrain(angle, 0, 180);
  return angle;
}

const char* stateToString(MastState state) {
  switch (state) {
    case MAST_RAISED:  return "RAISED";
    case MAST_LOWERED: return "LOWERED";
    case MAST_STOWED:  return "STOWED";
    default:           return "UNKNOWN";
  }
}

void printStatus() {
  Serial.print("STATUS: ");
  Serial.print(stateToString(mastState));
  Serial.print(", LED=");
  Serial.print(digitalRead(LED_PIN) ? "ON" : "OFF");
  Serial.print(", angle=");
  Serial.println(getServoAngleForState(mastState));
}