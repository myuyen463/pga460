#include <SPI.h>
#include <mcp_can.h>
#include <stdio.h>
#include <stdlib.h>

#define SS 53

int distance[4] = {0};
byte buf[8] = {0};
int counter = 1;
unsigned long timer;


MCP_CAN CAN(SS);

void setup()
{
  Serial.begin(19200);
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");
  delay(1000);
  timer = millis();
}

void loop()
{
  memset(buf,0, sizeof(buf));
  for(int i = 0; i<4; i++){
    distance[i] = random(1,1000);
    Serial.print(distance[i]); Serial.print("\t");
    switch(i){
      case 0:
        buf[0] = highByte(distance[i]);
        buf[1] = lowByte(distance[i]);
        break;
      case 1:
        buf[2] = highByte(distance[i]);
        buf[3] = lowByte(distance[i]);  
        break;
      case 2:
        buf[4] = highByte(distance[i]);
        buf[5] = lowByte(distance[i]);
        break;
      case 3:
        buf[6] = highByte(distance[i]);
        buf[7] = lowByte(distance[i]);
        break;
    }
  }
  counter++;
  if(millis()-timer >1000){
    Serial.print(counter);
    counter = 1;
    timer = millis();
  }
  Serial.println();
  CAN.sendMsgBuf(0x23, 0, strlen(buf), buf);
  delay(50);
}
