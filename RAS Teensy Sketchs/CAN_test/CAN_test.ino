#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10); 

byte data[8] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte dataspeed[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
byte data3[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Software kill switch
byte cool[8];

void setup() {
  Serial.begin(9600);  
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("MCP2515 Initializing...");
    delay(100);
  }
  Serial.println("Initialized MCP2515");
  CAN0.setMode(MCP_NORMAL); // Set to normal mode
}

void loop() {

  byte sendStatus = CAN0.sendMsgBuf(0x144, 0, 8, data);
  if (sendStatus == CAN_OK) {
    Serial.println("Message Sent Successfully");
  } else {
    Serial.println("Error Sending Message");
  }

  // Receive CAN message
  unsigned long canId;
  byte len;

  if (CAN0.checkReceive() == CAN_MSGAVAIL) { 
    CAN0.readMsgBuf(&canId, &len, cool);
    
    Serial.print("Received CAN ID: ");
    Serial.println(canId, HEX);
    Serial.print("Data: ");
    
    for (int i = 0; i < len; i++) {
      Serial.print(cool[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  delay(500); 
}
