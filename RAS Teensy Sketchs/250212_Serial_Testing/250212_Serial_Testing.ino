#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10); 

byte data[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte cool[8];
float initial5 = 0;
float current_value = 0;

void setup() {
  Serial.begin(9600);  
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("MCP2515 Initializing...");
    delay(100);
  }
  Serial.println("Initialized MCP2515");
  CAN0.setMode(MCP_NORMAL); // Set to normal mode

  byte sendStatus = CAN0.sendMsgBuf(0x145, 0, 8, data);

  // Receive CAN message
  unsigned long canId;
  byte len;

  delay(1);
  if (CAN0.checkReceive() == CAN_MSGAVAIL) { 
    CAN0.readMsgBuf(&canId, &len, cool);
  }
  // If we received enough data, set the initial offset
  if (len >= 8) {
    unsigned long convertedValue = convertData(cool, len);
    initial5 = convertedValue * 0.01;
    Serial.println("Initial Value");
    Serial.println(initial5);

}
  

}

unsigned long convertData(const byte *buffer, byte len) {
  if(len < 8) {
    return 0;
  }
  return ((unsigned long)buffer[7] << 24) |
         ((unsigned long)buffer[6] << 16) |
         ((unsigned long)buffer[5] << 8)  |
         ((unsigned long)buffer[4]);
}


void loop() {

  byte sendStatus = CAN0.sendMsgBuf(0x145, 0, 8, data);

  // Receive CAN message
  unsigned long canId;
  byte len;
  delay(1);

  if (CAN0.checkReceive() == CAN_MSGAVAIL) { 
    CAN0.readMsgBuf(&canId, &len, cool);
  }
  // If we received enough data, set the initial offset
  if (len >= 8) {
    unsigned long convertedValue = convertData(cool, len);
    current_value = convertedValue * 0.01;
    Serial.println("Degrees Value");
    Serial.println(current_value - initial5);

}
  

  delay(500); 
}