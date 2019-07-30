#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(53);

int counter = 1;
unsigned long timer;

void setup() {
  Serial.begin(19200);
  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ);
  mcp2515.setNormalMode();
  
  //Serial.println("------- CAN Read ----------");
  //Serial.println("ID  DLC   DATA");
  timer = millis();
}

void loop() {
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      
    //Serial.print(canMsg.can_id, HEX); // print ID
    //Serial.print(" "); 
    //Serial.print(canMsg.can_dlc, HEX); // print DLC
    //Serial.print(" ");
    
    for (int i = 0; i<4; i++)  {  // print the data
      int distance = canMsg.data[2*i]*256 + canMsg.data[2*i+1];
      Serial.print(distance);
      Serial.print("\t");
    }
       
    counter++;
    if(millis()- timer > 1000){
      Serial.print(counter);
      counter = 1;
      timer = millis();
    }
    Serial.println();   
  }

}
