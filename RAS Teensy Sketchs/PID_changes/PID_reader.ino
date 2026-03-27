#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);   // MCP2515 CS pin

// Read PID parameters command (0x30)
// DATA[0] = 0x30, DATA[1..7] = 0x00
byte readPidCmd[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte rxBuf[8];

const unsigned long MOTOR_IDS[] = {0x141, 0x142, 0x143, 0x144};
const int NUM_MOTORS = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

void printHexByte(byte b) {
  if (b < 0x10) Serial.print("0");
  Serial.print(b, HEX);
}

void printPidValues(unsigned long canId, byte *data, byte len) {
  Serial.print("Reply from CAN ID 0x");
  Serial.println(canId, HEX);

  if (len < 8) {
    Serial.println("  Invalid reply length");
    return;
  }

  if (data[0] != 0x30) {
    Serial.print("  Unexpected command byte: 0x");
    Serial.println(data[0], HEX);
    return;
  }

  // Per V3.9:
  // DATA[2] = Current loop KP
  // DATA[3] = Current loop KI
  // DATA[4] = Speed loop KP
  // DATA[5] = Speed loop KI
  // DATA[6] = Position loop KP
  // DATA[7] = Position loop KI
  Serial.print("  CurrKP: "); Serial.print(data[2]); Serial.print(" (0x"); printHexByte(data[2]); Serial.println(")");
  Serial.print("  CurrKI: "); Serial.print(data[3]); Serial.print(" (0x"); printHexByte(data[3]); Serial.println(")");
  Serial.print("  SpdKP : "); Serial.print(data[4]); Serial.print(" (0x"); printHexByte(data[4]); Serial.println(")");
  Serial.print("  SpdKI : "); Serial.print(data[5]); Serial.print(" (0x"); printHexByte(data[5]); Serial.println(")");
  Serial.print("  PosKP : "); Serial.print(data[6]); Serial.print(" (0x"); printHexByte(data[6]); Serial.println(")");
  Serial.print("  PosKI : "); Serial.print(data[7]); Serial.print(" (0x"); printHexByte(data[7]); Serial.println(")");

  Serial.print("  Raw Data: ");
  for (int i = 0; i < len; i++) {
    printHexByte(data[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
}

bool waitForPidReply(unsigned long expectedReplyId, unsigned long timeoutMs = 200) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN0.readMsgBuf(&canId, &len, rxBuf);

      // Only accept the expected reply ID for this request
      if (canId == expectedReplyId) {
        printPidValues(canId, rxBuf, len);
        return true;
      } else {
        // Print unexpected traffic for debugging
        Serial.print("Unexpected CAN ID 0x");
        Serial.print(canId, HEX);
        Serial.print(" Data: ");
        for (int i = 0; i < len; i++) {
          printHexByte(rxBuf[i]);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }

  return false;
}

void requestPidFromMotor(unsigned long txId) {
  unsigned long expectedReplyId = txId + 0x100;  // 0x141 -> 0x241, etc.

  Serial.print("Requesting PID from motor CAN ID 0x");
  Serial.println(txId, HEX);

  byte sendStatus = CAN0.sendMsgBuf(txId, 0, 8, readPidCmd);
  if (sendStatus == CAN_OK) {
    Serial.println("  Message sent successfully");
  } else {
    Serial.println("  Error sending message");
    Serial.println();
    return;
  }

  if (!waitForPidReply(expectedReplyId)) {
    Serial.print("  Timeout waiting for reply from 0x");
    Serial.println(expectedReplyId, HEX);
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("MCP2515 Initializing...");
    delay(100);
  }

  Serial.println("Initialized MCP2515");
  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN set to normal mode");
  Serial.println();
}

void loop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    requestPidFromMotor(MOTOR_IDS[i]);
    delay(100);
  }

  Serial.println("Finished one full PID scan of 0x141-0x144");
  Serial.println("==========================================");
  delay(2000);
}
