
#include <SoftwareSerial.h>

// P1 and P2 threshold values

//P1 threshold levels
#define P1_THR_0 0x0A
#define P1_THR_1 0x68
#define P1_THR_2 0x89
#define P1_THR_3 0x88
#define P1_THR_4 0x88
#define P1_THR_5 0x88
#define P1_THR_6 0x81
#define P1_THR_7 0xCA
#define P1_THR_8 0x54
#define P1_THR_9 0x20
#define P1_THR_10 0xE8
#define P1_THR_11 0x2A
#define P1_THR_12 0x2D
#define P1_THR_13 0x3C
#define P1_THR_14 0x32
#define P1_THR_15 0x00

//P2 threshold values
#define P2_THR_0 0x88
#define P2_THR_1 0x88
#define P2_THR_2 0x88
#define P2_THR_3 0x88
#define P2_THR_4 0x88
#define P2_THR_5 0x88
#define P2_THR_6 0x84
#define P2_THR_7 0x21
#define P2_THR_8 0x08
#define P2_THR_9 0x42
#define P2_THR_10 0x10
#define P2_THR_11 0x80
#define P2_THR_12 0x80
#define P2_THR_13 0x80
#define P2_THR_14 0x80
#define P2_THR_15 0x00

//SENSOR WRITE VALUES
#define USER_DATA1 0x00
#define USER_DATA2 0x00
#define USER_DATA3 0x00
#define USER_DATA4 0x00
#define USER_DATA5 0x00
#define USER_DATA6 0x00
#define USER_DATA7 0x00
#define USER_DATA8 0x00
#define USER_DATA9 0x00
#define USER_DATA10 0x00
#define USER_DATA11 0x00
#define USER_DATA12 0x00
#define USER_DATA13 0x00
#define USER_DATA14 0x00
#define USER_DATA15 0x00
#define USER_DATA16 0x00
#define USER_DATA17 0x00
#define USER_DATA18 0x00
#define USER_DATA19 0x00
#define USER_DATA20 0x00

#define INIT_GAIN 0x60
#define FREQUENCY 0x8F
#define DEADTIME 0xA0

#define PULSE_P1 0x08
#define PULSE_P2 0x10
#define CURR_LIM_P1 0x55
#define CURR_LIM_P2 0x55
#define REC_LENGTH 0x19
#define FREQ_DIAG 0x33
#define SAT_FDIAG_TH 0xEE
#define FVOLT_DEC 0x7C
#define DECPL_TEMP 0x4F
#define DSP_SCALE 0x00
#define TEMP_TRIM 0x00
#define P1_GAIN_CTRL 0x0A
#define P2_GAIN_CTRL 0x09


//TVGAIN(Time-varying gain)
#define TVGAIN0 0x88
#define TVGAIN1 0x88
#define TVGAIN2 0x88
#define TVGAIN3 0x82
#define TVGAIN4 0x08
#define TVGAIN5 0x20
#define TVGAIN6 0x80

// Sync byte
#define syncByte 0x55


// Analog-front end amplifier range...
//32-64db: 0xCF, 46-78db: 0x8F, 52-84db: 0x4F, 58-90db; 0x0F
#define AFEGAINRANGE 0x8F

//ONE-ADDRESS UART commands
#define P1BL 0x00
#define P2BL 0x01
//#define TNLM 0x04
#define UMR 0x05
#define SRR 0x09
#define SRW 0x0A
#define EEBW 0x0C
#define TVGBW 0x0E
#define THRBW 0x10


byte ChecksumInput[50]; //checksum array to hold values
byte SENSOR[46];  //SENSOR Bulk-write array write
byte THBUFF[35]; //Threshold buffer array for P1 and P2
byte AFEGAIN[5]; //AFEGAIN array
byte TVG[10]; //Threshold voltage gain array
byte UMRData[35]; //holds Ultrasonic measurement data i.e. data,width,amplitude


byte numObj = 1;
byte numSerial = 2;

uint8_t counter = 1;
unsigned long timer;
double minDistLim = 0.00;
uint16_t commandDelay = 5;
byte regAddr;

/*------------------------------------------------- initEEPROM -----
  |  Function initEEPROM
  |
  |  Purpose:  Updates user EEPROM values, and performs bulk EEPROM write.
  |
  |  Parameters:
  |
  |  Returns:  none
  -------------------------------------------------------------------*/
void initEEPROM()
{
  // Murata MA58MF14-7N
  SENSOR[0] = syncByte;
  SENSOR[1] = EEBW;
  SENSOR[2] = USER_DATA1;
  SENSOR[3] = USER_DATA2;
  SENSOR[4] = USER_DATA3;
  SENSOR[5] = USER_DATA4;
  SENSOR[6] = USER_DATA5;
  SENSOR[7] = USER_DATA6;
  SENSOR[8] = USER_DATA7;
  SENSOR[9] = USER_DATA8;
  SENSOR[10] = USER_DATA9;
  SENSOR[11] = USER_DATA10;
  SENSOR[12] = USER_DATA11;
  SENSOR[13] = USER_DATA12;
  SENSOR[14] = USER_DATA13;
  SENSOR[15] = USER_DATA14;
  SENSOR[16] = USER_DATA15;
  SENSOR[17] = USER_DATA16;
  SENSOR[18] = USER_DATA17;
  SENSOR[19] = USER_DATA18;
  SENSOR[20] = USER_DATA19;
  SENSOR[21] = USER_DATA20;
  SENSOR[22] = TVGAIN0;
  SENSOR[23] = TVGAIN1;
  SENSOR[24] = TVGAIN2;
  SENSOR[25] = TVGAIN3;
  SENSOR[26] = TVGAIN4;
  SENSOR[27] = TVGAIN5;
  SENSOR[28] = TVGAIN6;
  SENSOR[29] = INIT_GAIN;
  SENSOR[30] = FREQUENCY;
  SENSOR[31] = DEADTIME;
  SENSOR[32] = PULSE_P1;
  SENSOR[33] = PULSE_P2;
  SENSOR[34] = CURR_LIM_P1;
  SENSOR[35] = CURR_LIM_P2;
  SENSOR[36] = REC_LENGTH;
  SENSOR[37] = FREQ_DIAG;
  SENSOR[38] = SAT_FDIAG_TH;
  SENSOR[39] = FVOLT_DEC;
  SENSOR[40] = DECPL_TEMP;
  SENSOR[41] = DSP_SCALE;
  SENSOR[42] = TEMP_TRIM;
  SENSOR[43] = P1_GAIN_CTRL;
  SENSOR[44] = P2_GAIN_CTRL;
  SENSOR[45] = calcChecksum(EEBW);

  Serial1.write(SENSOR, sizeof(SENSOR)); // serial transmit master data for bulk SENSOR
  delay(10);

  return;

}

/*------------------------------------------------- initTVG -----
  |  Function initTVG
  |
  |  Purpose:  Updates time varying gain (TVG) range and mapping, and performs bulk TVG write
  |
  |  Parameters:
  |
  |  Returns:  none
  -------------------------------------------------------------------*/
void initTVG()
{
  // Initialise TVG values in array
  TVG[0] = syncByte;
  TVG[1] = TVGBW;
  TVG[2] = TVGAIN0;
  TVG[3] = TVGAIN1;
  TVG[4] = TVGAIN2;
  TVG[5] = TVGAIN3;
  TVG[6] = TVGAIN4;
  TVG[7] = TVGAIN5;
  TVG[8] = TVGAIN6;
  TVG[9] = calcChecksum(TVGBW);

  Serial1.write(TVG, sizeof(TVG)); // serial transmit master data for bulk TVG
  Serial2.write(TVG, sizeof(TVG));
  delay(100);
  return;
}
/*---------------------------------------------initAFEGAIN-----
  | Function initAFEGAIN
  |
  | Purpose: Updates analog-front end(AFE) gain range and mapping, and performs serial write
  |
  | Parameters:
  |
  | Return: none
*/
void initAFEGAIN()
{
  regAddr = 0x26; //register Address associated with AFEGAIN range

  AFEGAIN[0] = syncByte;
  AFEGAIN[1] = SRW;
  AFEGAIN[2] = regAddr;
  AFEGAIN[3] = AFEGAINRANGE;
  AFEGAIN[4] = calcChecksum(SRW);

  Serial1.write(AFEGAIN, sizeof(AFEGAIN));
  Serial2.write(AFEGAIN, sizeof(AFEGAIN));
  delay(100);
  return;
}
/*-------------------------------------printSensorMeas -----
  |  Function printSensorMeas
  |
  |  Purpose:  Converts time-of-flight readout to distance in meters.
  |    Width and amplitude data only available in UART or OWU mode.
  |
  |  Parameters:
  |    umr (IN) -- Ultrasonic measurement result look-up selector:
  |        Distance (m)  Width Amplitude
  |        --------------------------------
  |      Obj1    0   1   2
  |      Obj2    3   4   5
  |      Obj3    6   7   8
  |      Obj4    9   10    11
  |      Obj5    12    13    14
  |      Obj6    15    16    17
  |      Obj7    18    19    20
  |      Obj8    21    22    23
  |
  |  Returns:  double representation of distance (m), width (us), or amplitude (8-bit)
  -------------------------------------------------------------------*/
double printSensorMeas(byte umr)
{
  int speedSound = 343; // 343 degC at room temperature
  double objReturn = 0;
  double digitalDelay = 0.00005 * speedSound;
  uint16_t objDist = 0;

  switch (umr)
  {
    case 0 : //Obj1 Distance (m)
      {
        objDist = (UMRData[1] << 8) + UMRData[2];
        objReturn = (objDist / 2 * 0.000001 * speedSound) - digitalDelay;
        break;
      }
    
    case 1: //Obj2 Distance (m)
      {
      objDist = (UMRData[7] << 8) + UMRData[8];
      objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
      break;
      }
      
    case 2: //Obj3 Distance (m)
      {
      objDist = (UMRData[13] << 8) + UMRData[14];
      objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
      break;
      }
      
    case 3: //Obj4 Distance (m)
      {
      objDist = (UMRData[17] << 8) + UMRData[18];
      objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
      break;
      }
      
      
    default: Serial.println("ERROR - Invalid object result!"); break;
  }
  
  return objReturn;
}

/*------------------------------------------sensorEcho------------
   Function sensorEcho

   Purpose: Burst signals to be detected upon reflected by objects

   Parameters:
    preset(IN) -- determines which preset command is run
      P1BL - Preset 1 Burst + Listen command
      P2BL - Preset 2 Burst + Listen command

   Returns: none
*/

/*------------------------------------------pullSensorMeas -----
  |  Function pullSensorMeas
  |
  |  Purpose:  Read the ultrasonic measurement result data based on the last busrt and/or listen command issued.
  |
  |  Parameters:
  |
  |  Returns:  If measurement data successfully read, return true.
  -------------------------------------------------------------------*/
bool pullSensorMeas()
{
  //pga460SerialFlush();

  memset(UMRData, 0, sizeof(UMRData));
  byte pullMeas[3] = { syncByte, UMR, calcChecksum(UMR) };

  Serial1.write(pullMeas, sizeof(pullMeas)); //serial transmit master data to read ultrasonic measurement results
  delay(1);

  if (Serial1.available() < 5)
  {
    // the data didn't come in - handle the problem here
    Serial.println("ERROR - Did not receive measurement results!");
    return false;
  }
  else
  {
    for (int n = 0; n < (2 + (numObj * 4)); n++)
    {
      UMRData[n] = Serial1.read();
      delay(1);
    }
  }

  for (int n = 0; n < (2 + (numObj * 4)); n++)
  {
    Serial.print(UMRData[n]); Serial.print(" ");
  }
  return true;
}

/*-------------------------------------------- initThresholds -----
  |  Function initThresholds
  |
  |  Purpose:  Updates threshold mapping for both presets, and performs bulk threshold write
  |
  |  Parameters:
  |
  |  Returns:  none
  -------------------------------------------------------------------*/
void initThreshold()
{
  //Initialize Threshold buffer array
  THBUFF[0] = syncByte;
  THBUFF[1] = THRBW;
  THBUFF[2] = P1_THR_0;
  THBUFF[3] = P1_THR_1;
  THBUFF[4] = P1_THR_2;
  THBUFF[5] = P1_THR_3;
  THBUFF[6] = P1_THR_4;
  THBUFF[7] = P1_THR_5;
  THBUFF[8] = P1_THR_6;
  THBUFF[9] = P1_THR_7;
  THBUFF[10] = P1_THR_8;
  THBUFF[11] = P1_THR_9;
  THBUFF[12] = P1_THR_10;
  THBUFF[13] = P1_THR_11;
  THBUFF[14] = P1_THR_12;
  THBUFF[15] = P1_THR_13;
  THBUFF[16] = P1_THR_14;
  THBUFF[17] = P1_THR_15;
  THBUFF[18] = P2_THR_0;
  THBUFF[19] = P2_THR_1;
  THBUFF[20] = P2_THR_2;
  THBUFF[21] = P2_THR_3;
  THBUFF[22] = P2_THR_4;
  THBUFF[23] = P2_THR_5;
  THBUFF[24] = P2_THR_6;
  THBUFF[25] = P2_THR_7;
  THBUFF[26] = P2_THR_8;
  THBUFF[27] = P2_THR_9;
  THBUFF[28] = P2_THR_10;
  THBUFF[29] = P2_THR_11;
  THBUFF[30] = P2_THR_12;
  THBUFF[31] = P2_THR_13;
  THBUFF[32] = P2_THR_14;
  THBUFF[33] = P2_THR_15;
  THBUFF[34] = calcChecksum(THRBW);

  Serial.println("Init threshold");
  Serial1.write(THBUFF, sizeof(THBUFF)); // serial transmit master data for bulk threhsold
  Serial2.write(THBUFF, sizeof(THBUFF));
  delay(100);
  
  return;
}
/*------------------------------------------------- registerRead -----
  |  Function registerRead
  |
  |  Purpose:  Read single register data from PGA460
  |
  |  Parameters:
  |    addr (IN) -- PGA460 register address to read data from
  |
  |  Returns:  8-bit data read from register
  -------------------------------------------------------------------*/
byte registerRead(byte addr, HardwareSerial serial)
{
  byte data = 0x00;
  byte temp = 0;

  pga460SerialFlush(serial);

  byte readBUFF[4] = { syncByte, SRR, addr, calcChecksum(SRR) };

  serial.write(readBUFF, sizeof(readBUFF));
  delay(10);
  

  for (int n = 0; n < 3; n++)
  {
    if (n == 1)
    {
      data = serial.read(); // store read data
      delay(1);
      //Serial.print(data); Serial.print(" ");
    }
    else
    {
      temp = serial.read();
      delay(1);
      //Serial.print(temp); Serial.print(" ");
    }
  }
  Serial.println(data);
  return data;
}

/*------------------------------------------------- pga460SerialFlush -----
  |  Function pga460SerialFlush
  |
  |  Purpose:  Clears the MSP430's UART receiver buffer
  |
  |  Parameters:
  |    none
  |
  |  Returns: none
  -------------------------------------------------------------------*/
void pga460SerialFlush(HardwareSerial serial)
{
  delay(5);
  serial.flush();
  
  return;
}

/*------------------------------------------------- calcChecksum -----
  |  Function calcChecksum
  |
  |  Purpose:  Calculates the UART checksum value based on the selected command and the user EERPOM values associated with the command
  |    This function is not applicable to TCI mode.
  |
  |  Parameters:
  |    cmd (IN) -- the UART command for which the checksum should be calculated for
  |
  |  Returns: byte representation of calculated checksum value
  -------------------------------------------------------------------*/
byte calcChecksum(byte cmd)
{
  byte checksumLoops = 0;
  unsigned char DataCount = 0;
  unsigned int carry = 0;
  switch (cmd)
  {
    case P1BL :  // PRESET 1 BURST AND LISTEN - NUMOBJECT IS 1 CHECKSUM
    case P2BL :  // PRESET 2 BURST and LISTEN
      ChecksumInput[0] = cmd;
      ChecksumInput[1] = numObj;
      checksumLoops = 2;
      break;
    case UMR : // ULTRASONIC MEASUREMENT RESULT CHECKSUM
      ChecksumInput[0] = cmd;
      checksumLoops = 1;
      break;
    case SRW : //RW //REGISTER WRITE CHECKSUM
      ChecksumInput[0] = cmd;
      ChecksumInput[1] = regAddr; //0x26
      ChecksumInput[2] = AFEGAINRANGE; //gain_range=0x0F
      checksumLoops = 3;
      break;
    case SRR: //RR // REGISTER READ CHECKSUM
      ChecksumInput[0] = cmd;
      ChecksumInput[1] = regAddr;
      checksumLoops = 2;
      break;
    case TVGBW : //TVGBW //TIME VARYING GAIN BULK WRITE CHECKSUM
      ChecksumInput[0] = cmd;
      ChecksumInput[1] = TVGAIN0;
      ChecksumInput[2] = TVGAIN1;
      ChecksumInput[3] = TVGAIN2;
      ChecksumInput[4] = TVGAIN3;
      ChecksumInput[5] = TVGAIN4;
      ChecksumInput[6] = TVGAIN5;
      ChecksumInput[7] = TVGAIN6;
      checksumLoops = 8;
      break;
    case THRBW : //THRBW //THRESHOLD BULK WRITE CHECKSUM
      ChecksumInput[0] = cmd;
      ChecksumInput[1] = P1_THR_0;
      ChecksumInput[2] = P1_THR_1;
      ChecksumInput[3] = P1_THR_2;
      ChecksumInput[4] = P1_THR_3;
      ChecksumInput[5] = P1_THR_4;
      ChecksumInput[6] = P1_THR_5;
      ChecksumInput[7] = P1_THR_6;
      ChecksumInput[8] = P1_THR_7;
      ChecksumInput[9] = P1_THR_8;
      ChecksumInput[10] = P1_THR_9;
      ChecksumInput[11] = P1_THR_10;
      ChecksumInput[12] = P1_THR_11;
      ChecksumInput[13] = P1_THR_12;
      ChecksumInput[14] = P1_THR_13;
      ChecksumInput[15] = P1_THR_14;
      ChecksumInput[16] = P1_THR_15;
      ChecksumInput[17] = P2_THR_0;
      ChecksumInput[18] = P2_THR_1;
      ChecksumInput[19] = P2_THR_2;
      ChecksumInput[20] = P2_THR_3;
      ChecksumInput[21] = P2_THR_4;
      ChecksumInput[22] = P2_THR_5;
      ChecksumInput[23] = P2_THR_6;
      ChecksumInput[24] = P2_THR_7;
      ChecksumInput[25] = P2_THR_8;
      ChecksumInput[26] = P2_THR_9;
      ChecksumInput[27] = P2_THR_10;
      ChecksumInput[28] = P2_THR_11;
      ChecksumInput[29] = P2_THR_12;
      ChecksumInput[30] = P2_THR_13;
      ChecksumInput[31] = P2_THR_14;
      ChecksumInput[32] = P2_THR_15;
      checksumLoops = 33;
      break;
    case EEBW:
      ChecksumInput[0] = cmd;
      ChecksumInput[1] = USER_DATA1;
      ChecksumInput[2] = USER_DATA2;
      ChecksumInput[3] = USER_DATA3;
      ChecksumInput[4] = USER_DATA4;
      ChecksumInput[5] = USER_DATA5;
      ChecksumInput[6] = USER_DATA6;
      ChecksumInput[7] = USER_DATA7;
      ChecksumInput[8] = USER_DATA8;
      ChecksumInput[9] = USER_DATA9;
      ChecksumInput[10] = USER_DATA10;
      ChecksumInput[11] = USER_DATA11;
      ChecksumInput[12] = USER_DATA12;
      ChecksumInput[13] = USER_DATA13;
      ChecksumInput[14] = USER_DATA14;
      ChecksumInput[15] = USER_DATA15;
      ChecksumInput[16] = USER_DATA16;
      ChecksumInput[17] = USER_DATA17;
      ChecksumInput[18] = USER_DATA18;
      ChecksumInput[19] = USER_DATA19;
      ChecksumInput[20] = USER_DATA20;
      ChecksumInput[21] = TVGAIN0;
      ChecksumInput[22] = TVGAIN1;
      ChecksumInput[23] = TVGAIN2;
      ChecksumInput[24] = TVGAIN3;
      ChecksumInput[25] = TVGAIN4;
      ChecksumInput[26] = TVGAIN5;
      ChecksumInput[27] = TVGAIN6;
      ChecksumInput[28] = INIT_GAIN;
      ChecksumInput[29] = FREQUENCY;
      ChecksumInput[30] = DEADTIME;
      ChecksumInput[31] = PULSE_P1;
      ChecksumInput[32] = PULSE_P2;
      ChecksumInput[33] = CURR_LIM_P1;
      ChecksumInput[34] = CURR_LIM_P2;
      ChecksumInput[35] = REC_LENGTH;
      ChecksumInput[36] = FREQ_DIAG;
      ChecksumInput[37] = SAT_FDIAG_TH;
      ChecksumInput[38] = FVOLT_DEC;
      ChecksumInput[39] = DECPL_TEMP;
      ChecksumInput[40] = DSP_SCALE;
      ChecksumInput[41] = TEMP_TRIM;
      ChecksumInput[42] = P1_GAIN_CTRL;
      ChecksumInput[43] = P2_GAIN_CTRL;
      checksumLoops = 44;
      break;
    default: break;
  }

  carry = 0;

  for ( DataCount = 0; DataCount < checksumLoops; DataCount++)
  {
    if ((ChecksumInput[DataCount] + carry) < carry)
    {
      carry = carry + ChecksumInput[DataCount] + 1;
    }
    else
    {
      carry = carry + ChecksumInput[DataCount];
    }
    if (carry > 0xFF)
    {
      carry = carry - 255;
    }
  }

  carry = (~carry & 0x00FF);
  return carry;

}


void setup()
{
  Serial.begin(19200);
  Serial1.begin(19200, SERIAL_8N2);
  Serial2.begin(19200, SERIAL_8N2);

  
  initThreshold();
  //initEEPROM();
  initAFEGAIN();
  initTVG();

  Serial.print("UART_DIAG: ");
  regAddr = 0x1E;
  registerRead(regAddr, Serial1); //register Address for REC_LENGTH
  Serial.print(" ");
  registerRead(regAddr, Serial2);
  Serial.println();
  delay(10);

  Serial.print("REC_LENGTH: ");
  regAddr = 0x22;
  registerRead(regAddr, Serial1); //register Address for REC_LENGTH
  Serial.print(" ");
  registerRead(regAddr, Serial2);
  Serial.println();
  delay(10);
  
  Serial.print("THR_CRC_ERR: ");
  regAddr = 0x4C;
  registerRead(regAddr,Serial1); //register Address for THR_CRC_ERR
  Serial.print(" ");
  registerRead(regAddr, Serial2);
  Serial.println();

  Serial.println("\nInitialization done");

  timer = millis();

}

void loop() {
  // -+-+-+-+-+-+-+-+-+-+-  ISSUE SHORT-RANGE BURST+LSITEN COMMAND   -+-+-+-+-+-+-+-+-+-+- //
  pga460SerialFlush(Serial1); pga460SerialFlush(Serial2);
  byte echo[4] = {syncByte, P1BL, numObj, calcChecksum(P1BL)}; // prepare bufCmd with 0xFF placeholders
  Serial1.write(echo, sizeof(echo));
  Serial2.write(echo, sizeof(echo));
  delay(20); // maximum record length is 65ms, so delay with margin


  // -+-+-+-+-+-+-+-+-+-+- ISSUE PULLING-DATA COMMAND -+-+-+-+-+-+-+-+-+-+-+-//
  memset(UMRData, 0, sizeof(UMRData));
  byte pullMeas[3] = {syncByte, UMR, calcChecksum(UMR)};
  Serial1.write(pullMeas, sizeof(pullMeas));
  Serial2.write(pullMeas, sizeof(pullMeas));
  delay(5);

  if(Serial1.available()<5){
    Serial.println("ERROR");
  }
  else
  {
    for (int n = 0; n < (2 + (numObj * 4)); n++)
    {
      UMRData[n] = Serial1.read();
    }
  }

  if(Serial2.available()<5){
    Serial.println("ERROR");
  }
  else
  {
    for (int n = 6; n < (8 + (numObj * 4)); n++)
    {
      UMRData[n] = Serial2.read();
    }
  }
  
  for(uint8_t i = 0; i < numSerial; i++){
    double distance = printSensorMeas(i);
    delay(1);
    Serial.print("US "); Serial.print(i+1); Serial.print(": ");
    if (distance > minDistLim && distance < 11.2)  
    {
      Serial.print(distance);Serial.print(" ");
    }
    else
    {
      Serial.print("0.. ");
    }
  }
  Serial.println();
}
