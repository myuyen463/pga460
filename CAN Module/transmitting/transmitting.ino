#include <SPI.h>
#include <mcp_can.h>
#include <stdio.h>
#include <stdlib.h>

#define SS 53

int distance;
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
  delay(1000);
}

void loop()
{
  memset(buf,0, sizeof(buf));
  distance = random(1,1000); 
  Serial.println(distance); 
  buf[0] = highByte(distance);
  buf[1] = lowByte(distance);
  CAN.sendMsgBuf(0x43, 0, strlen(buf), buf);
  delay(50);
}
