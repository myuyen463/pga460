#include <SPI.h>
#include "mcp_can.h"

const int spiCSPin = 53;
unsigned long timer;
int counter = 1;

unsigned char len = 0;
byte buf[8] = {0};
unsigned int distance[4] = {0};
MCP_CAN CAN(spiCSPin);

void setup()
{
  Serial.begin(19200);

  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz))
  {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS  Init OK!");
  delay(1000);
  timer = millis();
}


void loop()
{


  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);

    //unsigned long canId = CAN.getCanId();

    //Serial.print("Data from ID: 0x");
    //Serial.println(canId, HEX);

    for(int i=0; i<4; i++){
      distance[i] = buf[2*i]*256 + buf[2*i+1];
      Serial.print(distance[i]); Serial.print("\t");
    }
    counter++;
    if(millis()-timer >1000){
      Serial.print(counter);
      counter = 1;
      timer - millis();
    }
    Serial.println();
  }
}
