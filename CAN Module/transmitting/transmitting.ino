#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
//struct can_frame canMsg2;
MCP2515 mcp2515(53);
int counter = 1;
unsigned long timer;

void setup() {
  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;

  /*canMsg2.can_id  = 0x036;
    canMsg2.can_dlc = 8;
    canMsg2.data[0] = 0x0E;
    canMsg2.data[1] = 0x00;
    canMsg2.data[2] = 0x00;
    canMsg2.data[3] = 0x08;
    canMsg2.data[4] = 0x01;
    canMsg2.data[5] = 0x00;
    canMsg2.data[6] = 0x00;
    canMsg2.data[7] = 0xA0;*/

  while (!Serial);
  Serial.begin(19200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Example: Write to CAN");
  timer = millis();
}

void loop() {

  for (int i = 0; i < 4; i++) {
    int distance = random(1, 1000);
    switch (i) {
      case 0:
        canMsg1.data[0] = highByte(distance);
        canMsg1.data[1] = lowByte(distance);
        Serial.print(distance); Serial.print("\t");
        break;
      case 1:
        canMsg1.data[2] = highByte(distance);
        canMsg1.data[3] = lowByte(distance);
        Serial.print(distance); Serial.print("\t");
        break;
      case 2:
        canMsg1.data[4] = highByte(distance);
        canMsg1.data[5] = lowByte(distance);
        Serial.print(distance); Serial.print("\t");
        break;
      case 3:
        counter++;
        canMsg1.data[6] = highByte(counter);
        canMsg1.data[7] = lowByte(counter);
        Serial.print(counter);
        break;
    }
  }
  /*for (int i = 0; i < 8; i++) {
    Serial.print(canMsg1.data[i]); Serial.print("\t");
  }*/
  /*counter++;
  if(millis() - timer >1000){
    Serial.print(counter);
    counter = 1;
    timer = millis();
  }*/
  Serial.println();

  mcp2515.sendMessage(&canMsg1);
  //mcp2515.sendMessage(&canMsg2);
  delay(10);
}
