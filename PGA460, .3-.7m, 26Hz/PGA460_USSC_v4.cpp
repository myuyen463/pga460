/*
	PGA460_USSC.cpp

	BSD 2-clause "Simplified" License
	Copyright (c) 2017, Texas Instruments
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY TEXAS INSTRUMENTS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL TEXAS INSTRUMENTS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	The views and conclusions contained in the software and documentation are those
	of the authors and should not be interpreted as representing official policies,
	either expressed or implied, of the FreeBSD Project.

	Last Updated: Nov 2017
	By: A. Whitehead <make@energia.nu>
*/

#include "PGA460_USSC.h"
//#include "PGA460_SPI.h"
#include "Arduino.h"

/*------------------------------------------------- Global Variables -----
 |  Global Variables
 |
 |  Purpose:  Variables shared throughout the PGA460_USSC.cpp functions
 *-------------------------------------------------------------------*/
#pragma region globals
 // Pin mapping of BOOSTXL-PGA460 to LaunchPad by pin name



// Serial read timeout in milliseconds
#define MAX_MILLIS_TO_WAIT 250

// Define UART commands by name
	// Single Address
byte P1BL = 0x00;
byte P2BL = 0x01;
byte P1LO = 0x02;
byte P2LO = 0x03;
byte TNLM = 0x04;
byte UMR = 0x05;
byte TNLR = 0x06;
byte TEDD = 0x07;
byte SD = 0x08;
byte SRR = 0x09;
byte SRW = 0x0A;
byte EEBR = 0x0B;
byte EEBW = 0x0C;
byte TVGBR = 0x0D;
byte TVGBW = 0x0E;
byte THRBR = 0x0F;
byte THRBW = 0x10;
//Broadcast
byte BC_P1BL = 0x11;
byte BC_P2BL = 0x12;
byte BC_P1LO = 0x13;
byte BC_P2LO = 0x14;
byte BC_TNLM = 0x15;
byte BC_RW = 0x16;
byte BC_EEBW = 0x17;
byte BC_TVGBW = 0x18;
byte BC_THRBW = 0x19;
//CMDs 26-31 are reserved

// List user registers by name with default settings from TI factory
byte USER_DATA1 = 0x00;
byte USER_DATA2 = 0x00;
byte USER_DATA3 = 0x00;
byte USER_DATA4 = 0x00;
byte USER_DATA5 = 0x00;
byte USER_DATA6 = 0x00;
byte USER_DATA7 = 0x00;
byte USER_DATA8 = 0x00;
byte USER_DATA9 = 0x00;
byte USER_DATA10 = 0x00;
byte USER_DATA11 = 0x00;
byte USER_DATA12 = 0x00;
byte USER_DATA13 = 0x00;
byte USER_DATA14 = 0x00;
byte USER_DATA15 = 0x00;
byte USER_DATA16 = 0x00;
byte USER_DATA17 = 0x00;
byte USER_DATA18 = 0x00;
byte USER_DATA19 = 0x00;
byte USER_DATA20 = 0x00;
byte TVGAIN0 = 0xAF;
byte TVGAIN1 = 0xFF;
byte TVGAIN2 = 0xFF;
byte TVGAIN3 = 0x2D;
byte TVGAIN4 = 0x68;
byte TVGAIN5 = 0x36;
byte TVGAIN6 = 0xFC;
byte INIT_GAIN = 0xC0;
byte FREQUENCY = 0x8C;
byte DEADTIME = 0x00;
byte PULSE_P1 = 0x01;
byte PULSE_P2 = 0x12;
byte CURR_LIM_P1 = 0x47;
byte CURR_LIM_P2 = 0xFF;
byte REC_LENGTH = 0x1C;
byte FREQ_DIAG = 0x00;
byte SAT_FDIAG_TH = 0xEE;
byte FVOLT_DEC = 0x7C;
byte DECPL_TEMP = 0x0A;
byte DSP_SCALE = 0x00;
byte TEMP_TRIM = 0x00;
byte P1_GAIN_CTRL = 0x00;
byte P2_GAIN_CTRL = 0x00;
byte EE_CRC = 0xFF;
byte EE_CNTRL = 0x00;
byte P1_THR_0 = 0x88;
byte P1_THR_1 = 0x88;
byte P1_THR_2 = 0x88;
byte P1_THR_3 = 0x88;
byte P1_THR_4 = 0x88;
byte P1_THR_5 = 0x88;
byte P1_THR_6 = 0x84;
byte P1_THR_7 = 0x21;
byte P1_THR_8 = 0x08;
byte P1_THR_9 = 0x42;
byte P1_THR_10 = 0x10;
byte P1_THR_11 = 0x80;
byte P1_THR_12 = 0x80;
byte P1_THR_13 = 0x80;
byte P1_THR_14 = 0x80;
byte P1_THR_15 = 0x80;
byte P2_THR_0 = 0x88;
byte P2_THR_1 = 0x88;
byte P2_THR_2 = 0x88;
byte P2_THR_3 = 0x88;
byte P2_THR_4 = 0x88;
byte P2_THR_5 = 0x88;
byte P2_THR_6 = 0x84;
byte P2_THR_7 = 0x21;
byte P2_THR_8 = 0x08;
byte P2_THR_9 = 0x42;
byte P2_THR_10 = 0x10;
byte P2_THR_11 = 0x80;
byte P2_THR_12 = 0x80;
byte P2_THR_13 = 0x80;
byte P2_THR_14 = 0x80;
byte P2_THR_15 = 0x80;

// Miscellaneous variables; (+) indicates OWU transmitted byte offset
byte checksum = 0x00; 			// UART checksum value	
byte ChecksumInput[44]; 		// data byte array for checksum calculator
byte ultraMeasResult[34 + 3]; 	// data byte array for cmd5 and tciB+L return
byte diagMeasResult[5 + 3]; 		// data byte array for cmd8 and index1 return
byte tempNoiseMeasResult[4 + 3]; 	// data byte array for cmd6 and index0&1 return
byte echoDataDump[130 + 3]; 		// data byte array for cmd7 and index12 return
byte tempOrNoise = 0; 			// data byte to determine if temp or noise measurement is to be performed
byte comm = 0; 					// indicates UART (0), TCI (1), OWU (2) communication mode	
unsigned long starttime; 		// used for function time out
byte bulkThr[34 + 3];				// data byte array for bulk threhsold commands
//UART & OWU exclusive variables
byte syncByte = 0x55; 		// data byte for Sync field set UART baud rate of PGA460
byte regAddr = 0x00; 		// data byte for Register Address
byte regData = 0x00; 		// data byte for Register Data
byte uartAddr = 0; 			// PGA460 UART device address (0-7). '0' is factory default address
byte numObj = 1; 			// number of objects to detect
//OWU exclusive variables
signed int owuShift = 0;	// accoutns for OWU receiver buffer offset for capturing master transmitted data - always 0 for standard two-wire UART
		// SPI MISO receive data buffer for all commands	
#pragma endregion globals

/*------------------------------------------------- PGA460 Top Level -----
 |  PGA460 Top Level Scope Resolution Operator
 |
 | Use the double colon operator (::) to qualify a C++ member function, a top
 | level function, or a variable with global scope with:
 | • An overloaded name (same name used with different argument types)
 | • An ambiguous name (same name used in different classes)
 *-------------------------------------------------------------------*/
pga460::pga460() {}

/*------------------------------------------------- initBoostXLPGA460 -----
 |  Function initBoostXLPGA460
 |
 |  Purpose:  Configure the master communication mode and BOOSTXL-PGA460 hardware to operate in UART, TCI, or OWU mode.
 |  Configures master serial baud rate for UART/OWU modes. Updates UART address based on sketch input.
 |
 |  Parameters:
 |		mode (IN) -- sets communicaiton mode.
 |			0=UART
 |			1=TCI
 |			2=OWU
 |			3-SPI (Synchronous Mode)
 |			4 = Not Used
 |			5 = Not Used
 |			6=Bus_Demo_Bulk_TVG_or_Threshold_Broadcast_is_True
 |			7=Bus_Demo_UART_Mode
 |			8=Bus_Demo_OWU_One_Time_Setup
 |			9=Bus_Demo_OWU_Mode
 | 		baud (IN) -- PGA460 accepts a baud rate of 9600 to 115.2k bps
 | 		uartAddrUpdate (IN) -- PGA460 address range from 0 to 7
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initBoostXLPGA460(byte mode, uint32_t baud, byte uartAddrUpdate)
{
	// check for valid UART address
	if (uartAddrUpdate > 7)
	{
		uartAddrUpdate = 0; // default to '0'
		Serial.println("ERROR - Invalid UART Address!");
	}
	// globally update target PGA460 UART address and commands	
	if (uartAddr != uartAddrUpdate)
	{
		// Update commands to account for new UART addr
		  // Single Address
		P1BL = 0x00 + (uartAddrUpdate << 5);
		P2BL = 0x01 + (uartAddrUpdate << 5);
		P1LO = 0x02 + (uartAddrUpdate << 5);
		P2LO = 0x03 + (uartAddrUpdate << 5);
		TNLM = 0x04 + (uartAddrUpdate << 5);
		UMR = 0x05 + (uartAddrUpdate << 5);
		TNLR = 0x06 + (uartAddrUpdate << 5);
		TEDD = 0x07 + (uartAddrUpdate << 5);
		SD = 0x08 + (uartAddrUpdate << 5);
		SRR = 0x09 + (uartAddrUpdate << 5);
		SRW = 0x0A + (uartAddrUpdate << 5);
		EEBR = 0x0B + (uartAddrUpdate << 5);
		EEBW = 0x0C + (uartAddrUpdate << 5);
		TVGBR = 0x0D + (uartAddrUpdate << 5);
		TVGBW = 0x0E + (uartAddrUpdate << 5);
		THRBR = 0x0F + (uartAddrUpdate << 5);
		THRBW = 0x10 + (uartAddrUpdate << 5);
	}
	uartAddr = uartAddrUpdate;

	// turn on LP's Red LED to indicate code has started to run


	// set communication mode flag
	//if (mode < 4) // 0=UART, 1=TCI, 2=OWU, 3=SPI
	//{
		comm = mode;
		// disable synchronous mode dump to external memory

	/*}
	else if (mode == 6)
	{
		comm = 6; // bus demo user input mode only, and threshold or TVG bulk write broadcast commands are true
	}
	else if ((mode == 7) || (mode == 9))
	{
		comm = mode - 7; // bus demo only for either UART or OWU mode
	}
	else
	{
		comm = 99; // invalid communication type
	}*/

	//switch (mode)
	//{
	//case 0: // UART Mode
		// enable PGA460 UART communication mode
		//pinMode(COM_PD, OUTPUT);  digitalWrite(COM_PD, LOW);
		//pinMode(COM_SEL, OUTPUT);  digitalWrite(COM_SEL, LOW);
		Serial.begin(baud); 	// initialize COM UART serial channel
		Serial1.begin(baud, SERIAL_8N2);	// initialize PGA460 UART serial channel
		//break;
	/*case 1: //TCI Mode	
		// enable PGA460 TCI communication mode
		pinMode(COM_PD, OUTPUT);  digitalWrite(COM_PD, LOW);
		pinMode(COM_SEL, OUTPUT);  digitalWrite(COM_SEL, LOW);
		pinMode(TCI_TX, OUTPUT); digitalWrite(TCI_TX, HIGH);
		pinMode(TCI_RX, INPUT_PULLUP);
		Serial.begin(baud); 	// initialize COM UART serial channel
		Serial1.begin(baud);	// initialize PGA460 UART serial channel
		break;
	case 2: //OWU setup (part I)
		// enable PGA460 UART communication mode
		pinMode(COM_PD, OUTPUT);  digitalWrite(COM_PD, LOW);
		pinMode(COM_SEL, OUTPUT);  digitalWrite(COM_SEL, LOW);
		Serial.begin(baud); 	// initialize COM UART serial channel
		Serial1.begin(baud);	// initialize PGA460 UART serial channel
		PULSE_P1 = 0x80 | PULSE_P1; // update IO_IF_SEL bit to '1' for OWU mode for bulk EEPROM write
		break;


	default: break;
	}

	//OWU setup (part II)
	if ((comm == 2) || (mode == 8)) // mode8 is for one time setup of OWU per slave device for bus demo
	{
		// UART write to register PULSE_P1 (addr 0x1E) to set device into OWU mode
		regAddr = 0x1E;
		regData = PULSE_P1;
		byte buf10[5] = { syncByte, SRW, regAddr, regData, calcChecksum(SRW) };
		Serial1.write(buf10, sizeof(buf10));
		delay(50);

		// enable PGA460 OWU communication mode

	}*/


	return;
}

/*------------------------------------------------- defaultPGA460 -----
 |  Function defaultPGA460
 |
 |  Purpose:  Updates user EEPROM values, and performs bulk EEPROM write.
 |
 |  Parameters:
 |		xdcr (IN) -- updates user EEPROM based on predefined listing for a specific transducer.
 |			Modify existing case statements, or append additional case-statement for custom user EEPROM configurations.
 |			• 0 = Murata MA58MF14-7N
 |			• 1 = Murata MA40H1S-R
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::defaultPGA460(byte xdcr)
{
	switch (xdcr)
	{
	case 0: // Murata MA58MF14-7N
		USER_DATA1 = 0x00;
		USER_DATA2 = 0x00;
		USER_DATA3 = 0x00;
		USER_DATA4 = 0x00;
		USER_DATA5 = 0x00;
		USER_DATA6 = 0x00;
		USER_DATA7 = 0x00;
		USER_DATA8 = 0x00;
		USER_DATA9 = 0x00;
		USER_DATA10 = 0x00;
		USER_DATA11 = 0x00;
		USER_DATA12 = 0x00;
		USER_DATA13 = 0x00;
		USER_DATA14 = 0x00;
		USER_DATA15 = 0x00;
		USER_DATA16 = 0x00;
		USER_DATA17 = 0x00;
		USER_DATA18 = 0x00;
		USER_DATA19 = 0x00;
		USER_DATA20 = 0x00;
		TVGAIN0 = 0xAA;
		TVGAIN1 = 0xAA;
		TVGAIN2 = 0xAA;
		TVGAIN3 = 0x82;
		TVGAIN4 = 0x08;
		TVGAIN5 = 0x20;
		TVGAIN6 = 0x80;
		INIT_GAIN = 0x60;
		FREQUENCY = 0x8F;
		DEADTIME = 0xA0;
		/*if (comm == 2)
		{
			PULSE_P1 = 0x80 | 0x04;
		}
		else
		{*/
			PULSE_P1 = 0x08;
		//}
		PULSE_P2 = 0x10;
		CURR_LIM_P1 = 0x55;
		CURR_LIM_P2 = 0x55;
		REC_LENGTH = 0x14;
		FREQ_DIAG = 0x33;
		SAT_FDIAG_TH = 0xEE;
		FVOLT_DEC = 0x7C;
		DECPL_TEMP = 0x4F;
		DSP_SCALE = 0x00;
		TEMP_TRIM = 0x00;
		P1_GAIN_CTRL = 0x0A;
		P2_GAIN_CTRL = 0x09;
		break;
	/*case 1: // Murata MA40H1SR
		USER_DATA1 = 0x00;
		USER_DATA2 = 0x00;
		USER_DATA3 = 0x00;
		USER_DATA4 = 0x00;
		USER_DATA5 = 0x00;
		USER_DATA6 = 0x00;
		USER_DATA7 = 0x00;
		USER_DATA8 = 0x00;
		USER_DATA9 = 0x00;
		USER_DATA10 = 0x00;
		USER_DATA11 = 0x00;
		USER_DATA12 = 0x00;
		USER_DATA13 = 0x00;
		USER_DATA14 = 0x00;
		USER_DATA15 = 0x00;
		USER_DATA16 = 0x00;
		USER_DATA17 = 0x00;
		USER_DATA18 = 0x00;
		USER_DATA19 = 0x00;
		USER_DATA20 = 0x00;
		TVGAIN0 = 0xAA;
		TVGAIN1 = 0xAA;
		TVGAIN2 = 0xAA;
		TVGAIN3 = 0x51;
		TVGAIN4 = 0x45;
		TVGAIN5 = 0x14;
		TVGAIN6 = 0x50;
		INIT_GAIN = 0x54;
		FREQUENCY = 0x32;
		DEADTIME = 0xA0;
		/if (comm == 2)
		{
			PULSE_P1 = 0x80 | 0x08;
		}
		else
		{
			PULSE_P1 = 0x08;
		}
		PULSE_P2 = 0x10;
		CURR_LIM_P1 = 0x40;
		CURR_LIM_P2 = 0x40;
		REC_LENGTH = 0x19;
		FREQ_DIAG = 0x33;
		SAT_FDIAG_TH = 0xEE;
		FVOLT_DEC = 0x7C;
		DECPL_TEMP = 0x4F;
		DSP_SCALE = 0x00;
		TEMP_TRIM = 0x00;
		P1_GAIN_CTRL = 0x09;
		P2_GAIN_CTRL = 0x09;
		break;
	case 2: // user custom
	{
		// insert custom user EEPROM listing
	}*/
	default: break;
	}

	//if ((comm == 0 || comm == 2 || comm == 3) && (comm != 6)) // USART or OWU mode and not busDemo6
	//{
		byte buf12[46] = { syncByte, EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
			USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14,
			USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
			TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
			PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
			DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(EEBW) };

		//if (comm == 0 || comm == 2) // UART or OWU mode
		//{
			Serial1.write(buf12, sizeof(buf12)); // serial transmit master data for bulk EEPROM
		//}

		delay(50);

		// Update targeted UART_ADDR to address defined in EEPROM bulk switch-case
		byte uartAddrUpdate = (PULSE_P2 >> 5) & 0x07;
		if (uartAddr != uartAddrUpdate)
		{
			// Update commands to account for new UART addr
			  // Single Address
			P1BL = 0x00 + (uartAddrUpdate << 5);
			P2BL = 0x01 + (uartAddrUpdate << 5);
			P1LO = 0x02 + (uartAddrUpdate << 5);
			P2LO = 0x03 + (uartAddrUpdate << 5);
			TNLM = 0x04 + (uartAddrUpdate << 5);
			UMR = 0x05 + (uartAddrUpdate << 5);
			TNLR = 0x06 + (uartAddrUpdate << 5);
			TEDD = 0x07 + (uartAddrUpdate << 5);
			SD = 0x08 + (uartAddrUpdate << 5);
			SRR = 0x09 + (uartAddrUpdate << 5);
			SRW = 0x0A + (uartAddrUpdate << 5);
			EEBR = 0x0B + (uartAddrUpdate << 5);
			EEBW = 0x0C + (uartAddrUpdate << 5);
			TVGBR = 0x0D + (uartAddrUpdate << 5);
			TVGBW = 0x0E + (uartAddrUpdate << 5);
			THRBR = 0x0F + (uartAddrUpdate << 5);
			THRBW = 0x10 + (uartAddrUpdate << 5);
		}
		uartAddr = uartAddrUpdate;
	/*}
	else if (comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		tciIndexRW(13, true);	// TCI index 13 write		
	}
	else
	{
		//do nothing
	}*/

	return;
}

/*------------------------------------------------- initThresholds -----
 |  Function initThresholds
 |
 |  Purpose:  Updates threshold mapping for both presets, and performs bulk threshold write
 |
 |  Parameters:
 |		thr (IN) -- updates all threshold levels to a fixed level based on specific percentage of the maximum level.
 |			All times are mid-code (1.4ms intervals).
 |			Modify existing case statements, or append additional case-statement for custom user threshold configurations.
 |			• 0 = 25% Levels 64 of 255
 |			• 1 = 50% Levels 128 of 255
 |			• 2 = 75% Levels 192 of 255
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initThresholds(byte thr)
{
	switch (thr)
	{
	case 0: //25% Levels 64 of 255
		P1_THR_0 = 0x88;
		P1_THR_1 = 0x88;
		P1_THR_2 = 0x88;
		P1_THR_3 = 0x88;
		P1_THR_4 = 0x88;
		P1_THR_5 = 0x88;
		P1_THR_6 = 0x42;
		P1_THR_7 = 0x10;
		P1_THR_8 = 0x84;
		P1_THR_9 = 0x21;
		P1_THR_10 = 0x08;
		P1_THR_11 = 0x40;
		P1_THR_12 = 0x40;
		P1_THR_13 = 0x40;
		P1_THR_14 = 0x40;
		P1_THR_15 = 0x00;
		P2_THR_0 = 0x88;
		P2_THR_1 = 0x88;
		P2_THR_2 = 0x88;
		P2_THR_3 = 0x88;
		P2_THR_4 = 0x88;
		P2_THR_5 = 0x88;
		P2_THR_6 = 0x42;
		P2_THR_7 = 0x10;
		P2_THR_8 = 0x84;
		P2_THR_9 = 0x21;
		P2_THR_10 = 0x08;
		P2_THR_11 = 0x40;
		P2_THR_12 = 0x40;
		P2_THR_13 = 0x40;
		P2_THR_14 = 0x40;
		P2_THR_15 = 0x00;
		break;

	case 1: //50% Level (midcode) 128 of 255
		P1_THR_0 = 0x88;
		P1_THR_1 = 0x88;
		P1_THR_2 = 0x88;
		P1_THR_3 = 0x88;
		P1_THR_4 = 0x88;
		P1_THR_5 = 0x88;
		P1_THR_6 = 0x84;
		P1_THR_7 = 0x21;
		P1_THR_8 = 0x08;
		P1_THR_9 = 0x42;
		P1_THR_10 = 0x10;
		P1_THR_11 = 0x80;
		P1_THR_12 = 0x80;
		P1_THR_13 = 0x80;
		P1_THR_14 = 0x80;
		P1_THR_15 = 0x00;
		P2_THR_0 = 0x88;
		P2_THR_1 = 0x88;
		P2_THR_2 = 0x88;
		P2_THR_3 = 0x88;
		P2_THR_4 = 0x88;
		P2_THR_5 = 0x88;
		P2_THR_6 = 0x84;
		P2_THR_7 = 0x21;
		P2_THR_8 = 0x08;
		P2_THR_9 = 0x42;
		P2_THR_10 = 0x10;
		P2_THR_11 = 0x80;
		P2_THR_12 = 0x80;
		P2_THR_13 = 0x80;
		P2_THR_14 = 0x80;
		P2_THR_15 = 0x00;
		break;

	case 2: //75% Levels 192 of 255
		P1_THR_0 = 0x88;
		P1_THR_1 = 0x88;
		P1_THR_2 = 0x88;
		P1_THR_3 = 0x88;
		P1_THR_4 = 0x88;
		P1_THR_5 = 0x88;
		P1_THR_6 = 0xC6;
		P1_THR_7 = 0x31;
		P1_THR_8 = 0x8C;
		P1_THR_9 = 0x63;
		P1_THR_10 = 0x18;
		P1_THR_11 = 0xC0;
		P1_THR_12 = 0xC0;
		P1_THR_13 = 0xC0;
		P1_THR_14 = 0xC0;
		P1_THR_15 = 0x00;
		P2_THR_0 = 0x88;
		P2_THR_1 = 0x88;
		P2_THR_2 = 0x88;
		P2_THR_3 = 0x88;
		P2_THR_4 = 0x88;
		P2_THR_5 = 0x88;
		P2_THR_6 = 0xC6;
		P2_THR_7 = 0x31;
		P2_THR_8 = 0x8C;
		P2_THR_9 = 0x63;
		P2_THR_10 = 0x18;
		P2_THR_11 = 0xC0;
		P2_THR_12 = 0xC0;
		P2_THR_13 = 0xC0;
		P2_THR_14 = 0xC0;
		P2_THR_15 = 0x00;
		break;

	case 3: //Custom
		P1_THR_0 = 0xB2;
		P1_THR_1 = 0x66;
		P1_THR_2 = 0x5A;
		P1_THR_3 = 0x98;
		P1_THR_4 = 0x88;
		P1_THR_5 = 0x88;
		P1_THR_6 = 0x61;
		P1_THR_7 = 0x0A;
		P1_THR_8 = 0x62;
		P1_THR_9 = 0x94;
		P1_THR_10 = 0x66;
		P1_THR_11 = 0x22;
		P1_THR_12 = 0x2C;
		P1_THR_13 = 0x28;
		P1_THR_14 = 0x3C;
		P1_THR_15 = 0x00;
		P2_THR_0 = 0x41;
		P2_THR_1 = 0x44;
		P2_THR_2 = 0x10;
		P2_THR_3 = 0x06;
		P2_THR_4 = 0x09;
		P2_THR_5 = 0x99;
		P2_THR_6 = 0xDD;
		P2_THR_7 = 0x4C;
		P2_THR_8 = 0x31;
		P2_THR_9 = 0x08;
		P2_THR_10 = 0x42;
		P2_THR_11 = 0x24;
		P2_THR_12 = 0x30;
		P2_THR_13 = 0x36;
		P2_THR_14 = 0x3C;
		P2_THR_15 = 0x00;
		break;

	case 4: //Custom
		P1_THR_0 = 0x0A;
		P1_THR_1 = 0x68;
		P1_THR_2 = 0x69;
		P1_THR_3 = 0x88;
		P1_THR_4 = 0x88;
		P1_THR_5 = 0x88;
		P1_THR_6 = 0x81;
		P1_THR_7 = 0xCA;
		P1_THR_8 = 0x54;
		P1_THR_9 = 0x20;
		P1_THR_10 = 0xE8;
		P1_THR_11 = 0x2A;
		P1_THR_12 = 0x2D;
		P1_THR_13 = 0x3C;
		P1_THR_14 = 0x32;
		P1_THR_15 = 0x00;
		P2_THR_0 = 0x88;
		P2_THR_1 = 0x88;
		P2_THR_2 = 0x88;
		P2_THR_3 = 0x88;
		P2_THR_4 = 0x88;
		P2_THR_5 = 0x88;
		P2_THR_6 = 0x84;
		P2_THR_7 = 0x21;
		P2_THR_8 = 0x08;
		P2_THR_9 = 0x42;
		P2_THR_10 = 0x10;
		P2_THR_11 = 0x80;
		P2_THR_12 = 0x80;
		P2_THR_13 = 0x80;
		P2_THR_14 = 0x80;
		P2_THR_15 = 0x00;
		break;

	default: break;
	}

	//if ((comm == 0 || comm == 2 || comm == 3) && (comm != 6)) 	// USART or OWU mode and not busDemo6
	//{
		byte buf16[35] = { syncByte, THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
			  P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
			  P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6,
			  P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
			  calcChecksum(THRBW) };
		//if (comm == 0 || comm == 2) // UART or OWU mode
		//{
			Serial.println("Init Thres");
			Serial1.write(buf16, sizeof(buf16)); // serial transmit master data for bulk threhsold
		//}


	/*}
	else if (comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		tciIndexRW(5, true); //TCI Threshold Preset 1 write
		tciIndexRW(6, true); //TCI Threshold Preset 2 write
	}
	else
	{
		//do nothing
	}*/

	delay(100);
	return;
}

/*------------------------------------------------- initTVG -----
 |  Function initTVG
 |
 |  Purpose:  Updates time varying gain (TVG) range and mapping, and performs bulk TVG write
 |
 |  Parameters:
 |		agr (IN) -- updates the analog gain range for the TVG.
 |			• 0 = 32-64dB
 |			• 1 = 46-78dB
 |			• 2 = 52-84dB
 |			• 3 = 58-90dB
 |		tvg (IN) -- updates all TVG levels to a fixed level based on specific percentage of the maximum level.
 |			All times are mid-code (2.4ms intervals).
 |			Modify existing case statements, or append additional case-statement for custom user TVG configurations
 |			• 0 = 25% Levels of range
 |			• 1 = 50% Levels of range
 |			• 2 = 75% Levels of range
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initTVG(byte agr, byte tvg)
{
	byte gain_range = 0x4F;
	// set AFE gain range
	switch (agr)
	{
	case 3: //58-90dB
		gain_range = 0x0F;
		break;
	case 2: //52-84dB
		gain_range = 0x4F;
		break;
	case 1: //46-78dB
		gain_range = 0x8F;
		break;
	case 0: //32-64dB
		gain_range = 0xCF;
		break;
	default: break;
	}

	//if ((comm == 0 || comm == 2 || comm == 3) && (comm != 6)) 	// USART or OWU mode and not busDemo6
	//{
		regAddr = 0x26;
		regData = gain_range;
		byte buf10[5] = { syncByte, SRW, regAddr, regData, calcChecksum(SRW) };
		//if (comm == 0 || comm == 2) // UART or OWU mode
		//{
			Serial1.write(buf10, sizeof(buf10));
		//}

	//}
	/*else if (comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		//TODO enable index 10 write
		//tciIndexRW(10, true);
	}
	else
	{
		//do nothing
	}*/

	//Set fixed AFE gain value
	switch (tvg)
	{
	case 0: //25% Level
		TVGAIN0 = 0x88;
		TVGAIN1 = 0x88;
		TVGAIN2 = 0x88;
		TVGAIN3 = 0x41;
		TVGAIN4 = 0x04;
		TVGAIN5 = 0x10;
		TVGAIN6 = 0x40;
		break;

	case 1: //50% Levels
		TVGAIN0 = 0x88;
		TVGAIN1 = 0x88;
		TVGAIN2 = 0x88;
		TVGAIN3 = 0x82;
		TVGAIN4 = 0x08;
		TVGAIN5 = 0x20;
		TVGAIN6 = 0x80;
		break;

	case 2: //75% Levels
		TVGAIN0 = 0x88;
		TVGAIN1 = 0x88;
		TVGAIN2 = 0x88;
		TVGAIN3 = 0xC3;
		TVGAIN4 = 0x0C;
		TVGAIN5 = 0x30;
		TVGAIN6 = 0xC0;
		break;

	default: break;
	}

	//if ((comm == 0 || comm == 2 || comm == 3) && (comm != 6)) 	// USART or OWU mode and not busDemo6
	//{
		byte buf14[10] = { syncByte, TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(TVGBW) };

		//if (comm == 0 || comm == 2) // UART or OWU mode
		//{
			Serial1.write(buf14, sizeof(buf14)); // serial transmit master data for bulk TVG
		//}

	/*}
	else if (comm == 6)
	{
		return;
	}
	else if (comm == 1) // TCI mode
	{
		tciIndexRW(8, true); //TCI bulk TVG write
	}
	else
	{
		//do nothing
	}*/

	return;
}

/*------------------------------------------------- ultrasonicCmd -----
 |  Function ultrasonicCmd
 |
 |  Purpose:  Issues a burst-and-listen or listen-only command based on the number of objects to be detected.
 |
 |  Parameters:
 |		cmd (IN) -- determines which preset command is run
 |			• 0 = Preset 1 Burst + Listen command
 |			• 1 = Preset 2 Burst + Listen command
 |			• 2 = Preset 1 Listen Only command
 |			• 3 = Preset 2 Listen Only command
 |		numObjUpdate (IN) -- PGA460 can capture time-of-flight, width, and amplitude for 1 to 8 objects.
 |			TCI is limited to time-of-flight measurement data only.
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::ultrasonicCmd(byte cmd, byte numObjUpdate)
{
	numObj = numObjUpdate; // number of objects to detect
	byte bufCmd[4] = { syncByte, 0xFF, numObj, 0xFF }; // prepare bufCmd with 0xFF placeholders

	
	switch (cmd)
	{
		// SINGLE ADDRESS		
	case 0: // Send Preset 1 Burst + Listen command
	{
		bufCmd[1] = P1BL;
		bufCmd[3] = calcChecksum(P1BL);
		break;
	}
	case 1: // Send Preset 2 Burst + Listen command
	{
		bufCmd[1] = P2BL;
		bufCmd[3] = calcChecksum(P2BL);
		break;
	}
	case 2: // Send Preset 1 Listen Only command
	{
		bufCmd[1] = P1LO;
		bufCmd[3] = calcChecksum(P1LO);
		break;
	}
	case 3: // Send Preset 2 Listen Only command
	{
		bufCmd[1] = P2LO;
		bufCmd[3] = calcChecksum(P2LO);
		break;
	}

	// BROADCAST
	/*case 17: // Send Preset 1 Burst + Listen Broadcast command
	{
		bufCmd[1] = BC_P1BL;
		bufCmd[3] = calcChecksum(BC_P1BL);
		break;
	}
	case 18: // Send Preset 2 Burst + Listen Broadcast command
	{
		bufCmd[1] = BC_P2BL;
		bufCmd[3] = calcChecksum(BC_P2BL);
		break;
	}
	case 19: // Send Preset 1 Listen Only Broadcast command
	{
		bufCmd[1] = BC_P1LO;
		bufCmd[3] = calcChecksum(BC_P1LO);
		break;
	}
	case 20: // Send Preset 2 Listen Only Broadcast command
	{
		bufCmd[1] = BC_P2LO;
		bufCmd[3] = calcChecksum(BC_P2LO);
		break;
	}
	*/
	default: return;
	}

	//if (comm != 1) // USART or OWU modes only
	//{
		//if (comm == 0 || comm == 2) // UART or OWU mode
		//{
			Serial1.write(bufCmd, sizeof(bufCmd)); // serial transmit master data to initiate burst and/or listen command
		//}

	/*}
	else if (comm == 1) // TCI mode
	{
		tciCommand(cmd); // send preset 1 or 2 burst-and-listen or listen-only command
		pga460::tciRecord(numObj); // log up to eight TCI object toggles
	}
	else
	{
		//do nothing
	}*/
			if (cmd == 0) delay(10);
			else delay(25);
	//delay(70); // maximum record length is 65ms, so delay with margin
	return;
}

/*------------------------------------------------- pullUltrasonicMeasResult -----
 |  Function pullUltrasonicMeasResult
 |
 |  Purpose:  Read the ultrasonic measurement result data based on the last busrt and/or listen command issued.
 |	Only applicable to UART and OWU modes.
 |
 |  Parameters:
 |		busDemo (IN) -- When true, do not print error message for a failed reading when running bus demo
 |
 |  Returns:  If measurement data successfully read, return true.
 *-------------------------------------------------------------------*/
bool pga460::pullUltrasonicMeasResult(bool busDemo)
{
		pga460SerialFlush();

		memset(ultraMeasResult, 0, sizeof(ultraMeasResult));

		
		byte buf5[3] = { syncByte, UMR, calcChecksum(UMR) };

		
			Serial1.write(buf5, sizeof(buf5));


			while (Serial1.available() < (5 + owuShift))
			{
				// wait in this loop until we either get +5 bytes of data, or 0.25 seconds have gone by
			}

			if ((Serial1.available() < (5 + owuShift)))
			{
				if (busDemo == false)
				{
					// the data didn't come in - handle the problem here
					Serial.println("ERROR - Did not receive measurement results!");
				}
				return false;
			}
			else
			{
				for (int n = 0; n < ((2 + (numObj * 4)) + owuShift); n++)
				{
					ultraMeasResult[n] = Serial1.read();
					//delay(1);
				}
			
			}
		
		return true;
	
}

/*------------------------------------------------- printUltrasonicMeasResult -----
 |  Function printUltrasonicMeasResult
 |
 |  Purpose:  Converts time-of-flight readout to distance in meters.
 |		Width and amplitude data only available in UART or OWU mode.
 |
 |  Parameters:
 |		umr (IN) -- Ultrasonic measurement result look-up selector:
 |				Distance (m)	Width	Amplitude
 |				--------------------------------
 |			Obj1		0		1		2
 |			Obj2		3		4		5
 |			Obj3		6		7		8
 |			Obj4		9		10		11
 |			Obj5		12		13		14
 |			Obj6		15		16		17
 |			Obj7		18		19		20
 |			Obj8		21		22		23
 |
 |  Returns:  double representation of distance (m), width (us), or amplitude (8-bit)
 *-------------------------------------------------------------------*/
double pga460::printUltrasonicMeasResult(byte umr)
{
	int speedSound = 343; // 343 degC at room temperature
	double objReturn = 0;
	double digitalDelay = 0.00005 * 343;
	uint16_t objDist = 0;
	uint16_t objWidth = 0;
	uint16_t objAmp = 0;

	switch (umr)
	{
	case 0: //Obj1 Distance (m)
	{
		/*Serial.print(ultraMeasResult[1]);
		Serial.print(" ");
		Serial.println(ultraMeasResult[2]);*/
		objDist = (ultraMeasResult[1] << 8) + ultraMeasResult[2];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 1: //Obj1 Width (us)
	{
		objWidth = ultraMeasResult[3];
		objReturn = objWidth * 16;
		break;
	}
	case 2: //Obj1 Peak Amplitude
	{
		objAmp = ultraMeasResult[4];
		objReturn = objAmp;
		break;
	}

	/*case 3: //Obj2 Distance (m)
	{
		objDist = (ultraMeasResult[5] << 8) + ultraMeasResult[6];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 4: //Obj2 Width (us)
	{
		objWidth = ultraMeasResult[7];
		objReturn = objWidth * 16;
		break;
	}
	case 5: //Obj2 Peak Amplitude
	{
		objAmp = ultraMeasResult[8];
		objReturn = objAmp;
		break;
	}

	case 6: //Obj3 Distance (m)
	{
		objDist = (ultraMeasResult[9] << 8) + ultraMeasResult[10];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 7: //Obj3 Width (us)
	{
		objWidth = ultraMeasResult[11];
		objReturn = objWidth * 16;
		break;
	}
	case 8: //Obj3 Peak Amplitude
	{
		objAmp = ultraMeasResult[12];
		objReturn = objAmp;
		break;
	}
	case 9: //Obj4 Distance (m)
	{
		objDist = (ultraMeasResult[13] << 8) + ultraMeasResult[14];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 10: //Obj4 Width (us)
	{
		objWidth = ultraMeasResult[15];
		objReturn = objWidth * 16;
		break;
	}
	case 11: //Obj4 Peak Amplitude
	{
		objAmp = ultraMeasResult[16];
		objReturn = objAmp;
		break;
	}
	case 12: //Obj5 Distance (m)
	{
		objDist = (ultraMeasResult[17] << 8) + ultraMeasResult[18];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 13: //Obj5 Width (us)
	{
		objWidth = ultraMeasResult[19];
		objReturn = objWidth * 16;
		break;
	}
	case 14: //Obj5 Peak Amplitude
	{
		objAmp = ultraMeasResult[20];
		objReturn = objAmp;
		break;
	}
	case 15: //Obj6 Distance (m)
	{
		objDist = (ultraMeasResult[21] << 8) + ultraMeasResult[22];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 16: //Obj6 Width (us)
	{
		objWidth = ultraMeasResult[23];
		objReturn = objWidth * 16;
		break;
	}
	case 17: //Obj6 Peak Amplitude
	{
		objAmp = ultraMeasResult[24];
		objReturn = objAmp;
		break;
	}
	case 18: //Obj7 Distance (m)
	{
		objDist = (ultraMeasResult[25] << 8) + ultraMeasResult[26];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 19: //Obj7 Width (us)
	{
		objWidth = ultraMeasResult[27];
		objReturn = objWidth * 16;
		break;
	}
	case 20: //Obj7 Peak Amplitude
	{
		objAmp = ultraMeasResult[28];
		objReturn = objAmp;
		break;
	}
	case 21: //Obj8 Distance (m)
	{
		objDist = (ultraMeasResult[29] << 8) + ultraMeasResult[30];
		objReturn = (objDist / 2 * 0.000001*speedSound) - digitalDelay;
		break;
	}
	case 22: //Obj8 Width (us)
	{
		objWidth = ultraMeasResult[31];
		objReturn = objWidth * 16;
		break;
	}
	case 23: //Obj8 Peak Amplitude
	{
		objAmp = ultraMeasResult[32];
		objReturn = objAmp;
		break;
	}*/
	default: Serial.println("ERROR - Invalid object result!"); break;
	}
	return objReturn;
}

/*------------------------------------------------- runEchoDataDump -----
 |  Function runEchoDataDump
 |
 |  Purpose:  Runs a preset 1 or 2 burst and or listen command to capture 128 bytes of echo data dump.
 |		Toggle echo data dump enable bit to enable/disable echo data dump mode.
 |
 |  Parameters:
 |		preset (IN) -- determines which preset command is run:
 |			• 0 = Preset 1 Burst + Listen command
 |			• 1 = Preset 2 Burst + Listen command
 |			• 2 = Preset 1 Listen Only command
 |			• 3 = Preset 2 Listen Only command
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::runEchoDataDump(byte preset)
{
	
		// enable Echo Data Dump bit
		regAddr = 0x40;
		regData = 0x80;
		byte buf10[5] = { syncByte, SRW, regAddr, regData, calcChecksum(SRW) };
		
			pga460SerialFlush();
			Serial1.write(buf10, sizeof(buf10));
		

		delay(10);

		// run preset 1 or 2 burst and or listen command
		pga460::ultrasonicCmd(preset, 1);

		// disbale Echo Data Dump bit
		regData = 0x00;
		buf10[3] = regData;
		buf10[4] = calcChecksum(SRW);
		
			Serial1.write(buf10, sizeof(buf10));
		

	
	return;
}

/*------------------------------------------------- pullEchoDataDump -----
 |  Function pullEchoDataDump
 |
 |  Purpose:  Read out 128 bytes of echo data dump (EDD) from latest burst and or listen command.
 |		For UART and OWU, readout individual echo data dump register values, instead in bulk.
 |		For TCI, perform index 12 read of all echo data dump values in bulk.
 |		TODO: Enable UART and OWU cmd7 transducer echo data dump bulk read.
 |
 |  Parameters:
 |		element (IN) -- element from the 128 byte EDD memory
 |
 |  Returns:  byte representation of EDD element value
 *-------------------------------------------------------------------*/
byte pga460::pullEchoDataDump(byte element)
{
	if (comm != 1 && comm != 3) // UART or OWU mode
	{
		if (element == 0)
		{
			byte temp = 0;
			pga460SerialFlush();

			if (comm == 2)
			{
				owuShift = 2; // OWU receive buffer offset to ignore transmitted data
			}
			else
			{
				owuShift = 0;
			}

			regAddr = 0x80; // start of EDD memory
			byte buf9[4] = { syncByte, SRR, regAddr, calcChecksum(SRR) };
			Serial1.write(buf9, sizeof(buf9)); // read first byte of EDD memory		

			for (int m = 0; m < 129; m++) // loop readout by iterating through EDD address range
			{
				buf9[2] = regAddr;
				buf9[3] = calcChecksum(SRR);
				Serial1.write(buf9, sizeof(buf9));
				delay(30);

				for (int n = 0; n < (3 + owuShift); n++)
				{
					if (n == (1 + owuShift))
					{
						echoDataDump[m] = Serial1.read();
					}
					else
					{
						temp = Serial1.read();
					}
				}
				regAddr++;
			}
		}
		return echoDataDump[element];
	}
	
	else
	{
		//do nothing
	}
	return 0xFF;
}

/*------------------------------------------------- runDiagnostics -----
 |  Function runDiagnostics
 |
 |  Purpose:  Runs a burst+listen command to capture frequency, decay, and voltage diagnostic.
 |		Runs a listen-only command to capture noise level.
 |		Captures die temperature of PGA460 device.
 |		Converts raw diagnostics to comprehensive units
 |
 |  Parameters:
 |		run (IN) -- issue a preset 1 burst-and-listen command
 |		diag (IN) -- diagnostic value to return:
 |			• 0 = frequency diagnostic (kHz)
 |			• 1 = decay period diagnostic (us)
 |			• 2 = die temperature (degC)
 |			• 3 = noise level (8bit)
 |
 |  Returns:  double representation of last captured diagnostic
 *-------------------------------------------------------------------*/
double pga460::runDiagnostics(byte run, byte diag)
{
	double diagReturn = 0;
	pga460SerialFlush();
	int elementOffset = 0; //Only non-zero for OWU mode.
	int owuShiftSysDiag = 0; // Only non-zero for OWU mode.

	if (comm != 1) // USART and OWU
	{
		if (comm == 2)
		{
			owuShift = 2; // OWU receive buffer offset to ignore transmitted data
			owuShiftSysDiag = 1;
		}

		if (run == 1) // issue  P1 burst+listen, and run system diagnostics command to get latest results
		{
			// run burst+listen command at least once for proper diagnostic analysis
			pga460::ultrasonicCmd(0, 1);	// always run preset 1 (short distance) burst+listen for 1 object for system diagnostic
			//Serial.println("Record 1: Init");

			delay(100); // record time length maximum of 65ms, so add margin
			pga460SerialFlush();

			byte buf8[3] = { syncByte, SD, calcChecksum(SD) };

			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				Serial1.write(buf8, sizeof(buf8)); //serial transmit master data to read system diagnostic results			

					/*Serial.print("Record 2: Serial status	");
					Serial.println(Serial1.available());
					Serial.print("record 3: Shift	");
					Serial.println((4 + owuShift - owuShiftSysDiag));
					Serial.print("owuShift:  ");
					Serial.println(owuShift);
					Serial.print("owuShiftDiag:  ");
					Serial.println(owuShiftSysDiag);*/

				starttime = millis();
				while ((Serial1.available() < (4 + owuShift - owuShiftSysDiag)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT))
				{
					// wait in this loop until we either get +4 bytes of data or 0.25 seconds have gone by
					//Serial.println("Record 4: Waiting..");
				}
				if (Serial1.available() < (4 + owuShift - owuShiftSysDiag))
				{
					// the data didn't come in - handle the problem here
					Serial.println("ERROR - Did not receive system diagnostics!");
				}
				else
				{
					for (int n = 0; n < (4 + owuShift - owuShiftSysDiag); n++)
					{
						diagMeasResult[n] = Serial1.read();
						//Serial.println("Record 5: loops");
					}
				}
			}

		}

		if (diag == 2) //run temperature measurement
		{
			tempOrNoise = 0; // temp meas
			byte buf4[4] = { syncByte, TNLM, tempOrNoise, calcChecksum(TNLM) };
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				Serial1.write(buf4, sizeof(buf4)); //serial transmit master data to run temp measurement
				delay(10);
				pga460SerialFlush();
				delay(10);
			}


			byte buf6[3] = { syncByte, TNLR, calcChecksum(TNLR) };
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				Serial1.write(buf6, sizeof(buf6)); //serial transmit master data to read temperature and noise results
			}

			delay(100);
		}

		if (diag == 3) // run noise level meas
		{
			tempOrNoise = 1; // noise meas
			byte buf4[4] = { syncByte, TNLM, tempOrNoise, calcChecksum(TNLM) };

			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				Serial1.write(buf4, sizeof(buf4)); //serial transmit master data to run noise level measurement (requires at least 8.2ms of post-delay)
			}


			delay(10);
			pga460SerialFlush();
			delay(10);

			byte buf6[3] = { syncByte, TNLR, calcChecksum(TNLR) }; //serial transmit master data to read temperature and noise results
			if (comm == 0 || comm == 2) // UART or OWU mode
			{
				Serial1.write(buf6, sizeof(buf6));
			}


			delay(100);
		}

		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			if (diag == 2 || diag == 3) // pull temp and noise level results
			{
				starttime = millis();
				while ((Serial1.available() < (4 + owuShift - owuShiftSysDiag)) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT))
				{
					// wait in this loop until we either get +4 bytes of data or 0.25 seconds have gone by
				}

				if (Serial1.available() < (4 + owuShift - owuShiftSysDiag))
				{
					// the data didn't come in - handle the problem here
					Serial.println("ERROR - Did not receive temp/noise!");
				}
				else
				{
					for (int n = 0; n < (4 + owuShift - owuShiftSysDiag); n++)
					{
						tempNoiseMeasResult[n] = Serial1.read();
					}

				}
			}
			elementOffset = owuShift - owuShiftSysDiag; // OWU only
		}

	}
	
	else
	{
		//do nothing
	}

	delay(100);

	switch (diag)
	{
	case 0: // convert to transducer frequency in kHz
	{
		diagReturn = (1 / (diagMeasResult[1 + elementOffset] * 0.0000005)) / 1000;
	}
	break;
	case 1: // convert to decay period time in us
	{
		diagReturn = diagMeasResult[2 + elementOffset] * 16;
	}
	break;
	case 2: //convert to temperature in degC
	{
		diagReturn = (tempNoiseMeasResult[1 + elementOffset] - 64) / 1.5;
	}
	break;
	case 3: //noise floor level
	{
		diagReturn = tempNoiseMeasResult[2 + elementOffset];
	}
	break;
	default: break;
	}

	return diagReturn;
}

/*------------------------------------------------- burnEEPROM -----
 |  Function burnEEPROM
 |
 |  Purpose:  Burns the EEPROM to preserve the working/shadow register values to EEPROM after power
 |		cycling the PGA460 device. Returns EE_PGRM_OK bit to determine if EEPROM burn was successful.
 |
 |  Parameters:
 |		none
 |
 |  Returns:  bool representation of EEPROM program success
 *-------------------------------------------------------------------*/
bool pga460::burnEEPROM()
{
	byte burnStat = 0;
	byte temp = 0;
	bool burnSuccess = false;

	if (comm != 1 || comm != 3)
	{

		// Write "0xD" to EE_UNLCK to unlock EEPROM, and '0' to EEPRGM bit at EE_CNTRL register
		regAddr = 0x40; //EE_CNTRL
		regData = 0x68;
		byte buf10[5] = { syncByte, SRW, regAddr, regData, calcChecksum(SRW) };
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			Serial1.write(buf10, sizeof(buf10));
		}


		delay(1);

		// Write "0xD" to EE_UNLCK to unlock EEPROM, and '1' to EEPRGM bit at EE_CNTRL register
		regAddr = 0x40; //EE_CNTRL
		regData = 0x69;
		buf10[2] = regAddr;
		buf10[3] = regData;
		buf10[4] = calcChecksum(SRW);
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			Serial1.write(buf10, sizeof(buf10));
		}

		delay(1000);


		// Read back EEPROM program status
		if (comm == 2)
		{
			owuShift = 1; // OWU receive buffer offset to ignore transmitted data
		}
		pga460SerialFlush();
		regAddr = 0x40; //EE_CNTRL
		byte buf9[4] = { syncByte, SRR, regAddr, calcChecksum(SRR) };
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			Serial1.write(buf9, sizeof(buf9));
		}

		delay(10);
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			for (int n = 0; n < 3; n++)
			{
				if (n == 1 - owuShift)
				{
					burnStat = Serial1.read(); // store EE_CNTRL data
				}
				else
				{
					temp = Serial1.read();
				}
			}
		}

	}
	
	else
	{
		//do nothing
	}

	if ((burnStat & 0x04) == 0x04) { burnSuccess = true; } // check if EE_PGRM_OK bit is '1'

	return burnSuccess;
}

/*------------------------------------------------- broadcast -----
 |  Function broadcast
 |
 |  Purpose:  Send a broadcast command to bulk write the user EEPROM, TVG, and/or Threshold values for all devices, regardless of UART_ADDR.
 |		Placehold for user EEPROM broadcast available. Note, all devices will update to the same UART_ADDR in user EEPROM broadcast command.
 |		This function is not applicable to TCI mode.
 |
 |  Parameters:
 |		eeBulk (IN) -- if true, broadcast user EEPROM
 |		tvgBulk (IN) -- if true, broadcast TVG
 |		thrBulk (IN) -- if true, broadcast Threshold
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::broadcast(bool eeBulk, bool tvgBulk, bool thrBulk)
{

	// TVG broadcast command:
	if (tvgBulk == true)
	{
		byte buf24[10] = { syncByte, BC_TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(BC_TVGBW) };
		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			Serial1.write(buf24, sizeof(buf24));
		}

		delay(10);
	}

	// Threshold broadcast command:
	if (thrBulk == true)
	{
		byte buf25[35] = { syncByte, BC_THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
		  P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
		  P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6,
		  P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
		  calcChecksum(BC_THRBW) };

		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			Serial1.write(buf25, sizeof(buf25));
		}

		delay(10);
	}

	// User EEPROM broadcast command (placeholder):
	if (eeBulk == true)
	{
		byte buf23[46] = { syncByte, BC_EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
			USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14,
			USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
			TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
			PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
			DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(BC_EEBW) };

		if (comm == 0 || comm == 2) // UART or OWU mode
		{
			Serial1.write(buf23, sizeof(buf23));
		}

		delay(50);
	}

	return;
}


/*------------------------------------------------- calcChecksum -----
 |  Function calcChecksum
 |
 |  Purpose:  Calculates the UART checksum value based on the selected command and the user EERPOM values associated with the command
 |		This function is not applicable to TCI mode.
 |
 |  Parameters:
 |		cmd (IN) -- the UART command for which the checksum should be calculated for
 |
 |  Returns: byte representation of calculated checksum value
 *-------------------------------------------------------------------*/
byte pga460::calcChecksum(byte cmd)
{
	int checksumLoops = 0;

	cmd = cmd & 0x001F; // zero-mask command address of cmd to select correct switch-case statement

	switch (cmd)
	{
	case 0: //P1BL
	case 1: //P2BL
	case 2: //P1LO
	case 3: //P2LO
	case 17: //BC_P1BL
	case 18: //BC_P2BL
	case 19: //BC_P1LO
	case 20: //BC_P2LO
		ChecksumInput[0] = cmd;
		ChecksumInput[1] = numObj;
		checksumLoops = 2;
		break;
	case 4: //TNLM
	case 21: //TNLM
		ChecksumInput[0] = cmd;
		ChecksumInput[1] = tempOrNoise;
		checksumLoops = 2;
		break;
	case 5: //UMR
	case 6: //TNLR
	case 7: //TEDD
	case 8: //SD
	case 11: //EEBR
	case 13: //TVGBR
	case 15: //THRBR
		ChecksumInput[0] = cmd;
		checksumLoops = 1;
		break;
	case 9: //RR
		ChecksumInput[0] = cmd;
		ChecksumInput[1] = regAddr;
		checksumLoops = 2;
		break;
	case 10: //RW
	case 22: //BC_RW
		ChecksumInput[0] = cmd;
		ChecksumInput[1] = regAddr;
		ChecksumInput[2] = regData;
		checksumLoops = 3;
		break;
	case 14: //TVGBW
	case 24: //BC_TVGBW
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
	case 16: //THRBW
	case 25: //BC_THRBW
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
	case 12: //EEBW
	case 23: //BC_EEBW
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

	if (ChecksumInput[0] < 17) //only re-append command address for non-broadcast commands.
	{
		ChecksumInput[0] = ChecksumInput[0] + (uartAddr << 5);
	}

	uint16_t carry = 0;

	for (int i = 0; i < checksumLoops; i++)
	{
		if ((ChecksumInput[i] + carry) < carry)
		{
			carry = carry + ChecksumInput[i] + 1;
		}
		else
		{
			carry = carry + ChecksumInput[i];
		}

		if (carry > 0xFF)
		{
			carry = carry - 255;
		}
	}

	carry = (~carry & 0x00FF);
	//Serial.print("Checksum: ");
	//Serial.println(carry);
	return carry;
}


/*------------------------------------------------- pga460SerialFlush -----
 |  Function pga460SerialFlush
 |
 |  Purpose:  Clears the MSP430's UART receiver buffer
 |
 |  Parameters:
 |		none
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::pga460SerialFlush()
{
	delay(10);
	Serial1.flush();
	while ((Serial1.available() > 0))// || (Serial1.read() < 0xFF)) 
	{
		char temp = Serial1.read();
		//Serial1.flush();
	}

	//redundant clear
	for (int i = 0; i < 10; i++)
	{
		while (Serial.available() > 0)
		{
			char k = Serial.read();
			delay(1);
		}
		delay(1);
	}

	Serial1.flush();
	return;
}


/*------------------------------------------------- registerRead -----
 |  Function registerRead
 |
 |  Purpose:  Read single register data from PGA460
 |
 |  Parameters:
 |		addr (IN) -- PGA460 register address to read data from
 |
 |  Returns:  8-bit data read from register
 *-------------------------------------------------------------------*/
byte pga460::registerRead(byte addr)
{
	byte data = 0x00;
	byte temp = 0;

	if (comm == 2)
	{
		owuShift = 1; // OWU receive buffer offset to ignore transmitted data
	}
	else
	{
		owuShift = 0;
	}

	pga460SerialFlush();

	regAddr = addr;
	byte buf9[4] = { syncByte, SRR, regAddr, calcChecksum(SRR) };
	//Serial.print("Checksum: ");
	//Serial.println(buf9[3]);
	if (comm == 0 || comm == 2) // UART or OWU mode
	{
		Serial1.write(buf9, sizeof(buf9));
	}

	delay(10);
	if (comm == 0 || comm == 2) // UART or OWU mode
	{
		for (int n = 0; n < 3; n++)
		{
			if (n == 1 - owuShift)
			{
				data = Serial1.read(); // store read data
				delay(1);
				Serial.print(data);
				Serial.print(" ");
			}
			else
			{
				temp = Serial1.read();
				delay(1);
			}
		}
	}
	return data;
}

/*------------------------------------------------- registerWrite -----
 |  Function registerWrite
 |
 |  Purpose:  Write single register data to PGA460
 |
 |  Parameters:
 |		addr (IN) -- PGA460 register address to write data to
 |		data (IN) -- 8-bit data value to write into register
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
byte pga460::registerWrite(byte addr, byte data)
{
	regAddr = addr;
	regData = data;
	byte buf10[5] = { syncByte, SRW, regAddr, regData, calcChecksum(SRW) };
	if (comm == 0 || comm == 2) // UART or OWU mode
	{
		Serial1.write(buf10, sizeof(buf10));
	}

	delay(10);
}



/*------------------------------------------------- thresholdBulkRead -----
 |  Function thresholdBulkRead
 |
 |  Purpose:  Bulk read all threshold times and levels
 |
 |  Parameters:
 |		preset (IN) -- which preset's threshold data to read
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::thresholdBulkRead(byte preset)
{
	byte n = 0;
	byte buf15[2] = { syncByte, THRBR };
	byte presetOffset = 0;
	byte addr = 0x5F; // beginning of threshold register space

	switch (comm)
	{
	case 0:
	case 2:
		if (preset == 2) //Preset 2 advances 16 address bytes
		{
			presetOffset = 16;
		}

		for (int n = 0; n < 16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(addr + presetOffset);
			addr++;
			//Serial.print(bulkThr[n + presetOffset]);
			//Serial.print(" ");

		}
		//Serial.println();

		// Threshold Bulk Read Command 15 too large for Serial1 receive buffer
		/*Serial1.write(buf15, sizeof(buf15));
		delay (300);
		while (Serial1.available() > 0)
		{
			bulkThr[n] = Serial1.read();
			n++;
		}*/

		break;

	case 1: //TCI
		//TODO
		break;

	case 3: //SPI
		//TODO
		break;

	default:
		break;
	}
}

/*------------------------------------------------- thresholdBulkWrite -----
 |  Function thresholdBulkWrite
 |
 |  Purpose:  Bulk write to all threshold registers
 |
 |  Parameters:
 |		p1ThrMap (IN) -- data byte array for 16 bytes of Preset 1 threhsold data
  |		p2ThrMap (IN) -- data byte array for 16 bytes of Preset 2 threhsold data
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::thresholdBulkWrite(byte *p1ThrMap, byte *p2ThrMap)
{
	//bulk write new threshold values
	
		byte buf16[35] = { syncByte,	THRBW, p1ThrMap[0], p1ThrMap[1], p1ThrMap[2], p1ThrMap[3], p1ThrMap[4], p1ThrMap[5],
			p1ThrMap[6], p1ThrMap[7], p1ThrMap[8], p1ThrMap[9], p1ThrMap[10], p1ThrMap[11], p1ThrMap[12],
			p1ThrMap[13], p1ThrMap[14], p1ThrMap[15],
			p2ThrMap[0], p2ThrMap[1], p2ThrMap[2], p2ThrMap[3], p2ThrMap[4], p2ThrMap[5],
			p2ThrMap[6], p2ThrMap[7], p2ThrMap[8], p2ThrMap[9], p2ThrMap[10], p2ThrMap[11], p2ThrMap[12],
			p2ThrMap[13], p2ThrMap[14], p2ThrMap[15],
			calcChecksum(THRBW) };
		
			Serial1.write(buf16, sizeof(buf16)); // serial transmit master data for bulk threhsold
		


	
	delay(100);
	return;
}

/*------------------------------------------------- eepromThreshold -----
 |  Function eepromThreshold
 |
 |  Purpose:  Copy a single preset's threshold times and levels
 |  			to USER_DATA1-16 in EEPROM
 |
 |  Parameters:
 |		preset (IN) -- preset's threshold to copy
 |		saveLoad (IN) -- when false, copy threshold to EEPROM;
 |					when true, copy threshold from EEPROM
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::eepromThreshold(byte preset, bool saveLoad)
{
	byte presetOffset = 0;
	byte addr = 0x5F; // beginning of threshold memory space

	if (saveLoad == false) // save thr
	{
		//Preset 2 advances 16 address bytes
		if (preset == 2 || preset == 4)
		{
			presetOffset = 16;
		}

		for (int n = 0; n < 16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(addr + presetOffset);
			// write threshold values into USER_DATA1-16
			registerWrite(n, bulkThr[n + presetOffset]);
			addr++;
		}
	}
	else // load thr
	{
		//Preset 2 advances 16 address bytes
		if (preset == 2 || preset == 4) //Preset 2 advances 16 address bytes
		{
			presetOffset = 16;
		}

		// copy USER_DATA1-16 into selected preset threhsold space
		for (int n = 0; n < 16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(n);
			// bulk write to threshold
			registerWrite(addr + presetOffset, bulkThr[n + presetOffset]);
			addr++;
		}
	}
}

/*------------------------------------------------- FUNCTION_NAME -----
 |  Function FUNCTION_NAME
 |
 |  Purpose:  EXPLAIN WHAT THIS FUNCTION DOES TO SUPPORT THE CORRECT
 |      OPERATION OF THE PROGRAM, AND HOW IT DOES IT.
 |
 |  Parameters:
 |      parameter_name (IN, OUT, or IN/OUT) -- EXPLANATION OF THE
 |              PURPOSE OF THIS PARAMETER TO THE FUNCTION.
 |                      (REPEAT THIS FOR ALL FORMAL PARAMETERS OF
 |                       THIS FUNCTION.
 |                       IN = USED TO PASS DATA INTO THIS FUNCTION,
 |                       OUT = USED TO PASS DATA OUT OF THIS FUNCTION
 |                       IN/OUT = USED FOR BOTH PURPOSES.)
 |
 |  Returns:  IF THIS FUNCTION SENDS BACK A VALUE VIA THE RETURN
 |      MECHANISM, DESCRIBE THE PURPOSE OF THAT VALUE HERE.
 *-------------------------------------------------------------------*/
