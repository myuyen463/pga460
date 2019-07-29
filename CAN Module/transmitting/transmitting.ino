#include <SPI.h>
#include <mcp_can.h>
#include <stdio.h>
#include <stdlib.h>

#define SS 53

int distance = 256;
byte buf[8] = {0};


MCP_CAN CAN(SS);

void setup()
{
  Serial.begin(19200);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");
  buf[0] = highByte(distance);
  buf[1] = lowByte(distance);
  Serial.print("DEC value: ");
  for(int i = 0; i<8; i++){
    Serial.print(buf[i]); Serial.print(" ");
  }
  unsigned int word = buf[0] * 256 + buf[1];
  Serial.print(word);
  

  

}

void loop()
{
  CAN.sendMsgBuf(0x43, 0, 8, buf);
  delay(1000);
}
