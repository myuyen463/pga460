#include <SPI.h>
#include <mcp_can.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define numPGAs 2
#define SS 53

uint8_t distance = 653;
byte buf[8] = {};


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
  itoa(distance, buf, 16);

}



void loop()
{
  Serial.println("In loop");
  CAN.sendMsgBuf(0x43, 0, 3, buf);
  delay(1000);
}
