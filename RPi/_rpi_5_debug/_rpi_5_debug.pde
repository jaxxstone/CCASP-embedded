/*
 * File: _rpi_5_debug.ino
 * Date: 11/22/2015
 * Author: DJ 
 * Desc: Program for Arduino-based (nano, pro mini, etc.) end-node for sensor data collection and driving relays and motors
 * Functions included:
 *    - serial_reliable_datagram_client
 *    - defining full set of values to be received from gw
 *    - works with printf, but bytes are set manually per variable received, don't like it (why struct does not work??)
 *    - writing to file (tab delimited)
 *    
 * Works on Linux and OSX. Build and test with:
 *    - tools/simBuild examples/serial/serial_reliable_datagram_client/_rpi_5_debug.pde
 *    - RH_HARDWARESERIAL_DEVICE_NAME=/dev/ttyUSB0 ./_rpi_5_debug
 *    
 * This debug version is for RPi
 */

#include <stdio.h>
#include <RHReliableDatagram.h>
#include <RH_Serial.h>

#include <stdlib.h>
#include<iostream>
using namespace std;

#define CLIENT_ADDRESS 251  // RPi is the client
#define SERVER_ADDRESS 252
 
#include <RHutil/HardwareSerial.h>
// On Linux we connect to a physical serial port
// You can override this with RH_HARDWARESERIAL_DEVICE_NAME environment variable
HardwareSerial hardwareserial("/dev/ttyUSB0");
RH_Serial driver(hardwareserial);

struct dataStruct{
  int nodeID;
  int temp; 
  int airHumid;
  int soilHumidPerc; 
  int rainPerc;
  bool fluidLevelHigh; 
  double latitude;
  double longitude;
  int proxCount;
  float volts;
}myData;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// First loop delay to allow Arduino to initialize through USB (skip the bootstrap)
int firstLoop = 1;  // set firstLoop to True

// Default polling interval in ms, can use the rpi program in continuous loop (default) or can invoke through shell script
int pollingInt = 10000; 

void setup() 
{
  Serial.begin(9600);
  // Configure the port RH_Serial will use:
  driver.serial().begin(9600);
  if (!manager.init())
    Serial.println("init failed");
}

uint8_t data[] = "2001024";  // in the format of endNodeID+Relay1+Relay2+PWM, endNodeID starts from 2
// Dont put this on the stack:
uint8_t buf[RH_SERIAL_MAX_MESSAGE_LEN];

FILE* fp;

void loop()
{
  if (firstLoop == 0){  // if firstLoop False then process lines inside if-loop
	  Serial.println("Sending to serial_reliable_datagram_server");
		
	  // Send a message to manager_server
	  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
	  {
		// Now wait for a reply from the server
		uint8_t len = sizeof(buf);
		uint8_t from;   
		if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
		{
		  Serial.print("got reply from : 0x");
		  Serial.print(from, HEX);
		  Serial.print(": ");
		  
		  cout << "" << endl;

		  printf("%d\n",buf[0] + buf[1]);  // int: 2 bytes
		  printf("%d\n",buf[2] + buf[3]);
		  printf("%d\n",buf[4] + buf[5]);
		  printf("%d\n",buf[6] + buf[7]);
		  printf("%d\n",buf[8] + buf[9]);		  
		  printf("%d\n",buf[10]);             // bool: 1 byte
		  printf("%d\n",buf[11] + buf[12] + buf[13] + buf[14]); // double=float: 4 bytes
		  printf("%d\n",buf[15] + buf[16] + buf[17] + buf[18]);
		  printf("%d\n",buf[19] + buf[20]); // int
		  printf("%d\n",buf[21] + buf[22] + buf[23] + buf[24]); // float

	//      fp = fopen ("inputfile.txt", "w+"); // overwriting on each new row written
		  fp = fopen ("inputfile.txt", "a+");   // appending to the file
		  fprintf(fp, "%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\n", buf[0] + buf[1], buf[2] + buf[3], 
			buf[4] + buf[5], buf[6] + buf[7], buf[8] + buf[9],
			buf[10], buf[11] + buf[12] + buf[13] + buf[14], buf[15] + buf[16] + buf[17] + buf[18], 
			buf[19] + buf[20], buf[21] + buf[22] + buf[23] + buf[24]);
		  fclose(fp);
				
	//    Message with a good checksum received, dump it.
	//      memcpy(&myData, buf, sizeof(myData));
	//
	//      Serial.print("Air temp : ");
	//      int ptemp = (int)(myData.temp);
	//      printf("Decimals: %d\n",ptemp);
		  
	//      Serial.print("Air humid : ");
	//      int pairHumid = (myData.airHumid);
	//      Serial.println(pairHumid);
	//      
	//      Serial.print("Soil humid : ");
	//      int psoilHumidPerc = (myData.soilHumidPerc);      
	//      Serial.println(psoilHumidPerc); 
	//           
	//      Serial.print("Rain qty % : ");  
	//      int prainPerc = (int)(myData.rainPerc);          
	//      Serial.println(prainPerc);
	//            
	//      Serial.print("Fluid overflow : ");
	//      int pfluidLevelHigh = (int)(myData.fluidLevelHigh);
	//      Serial.println(pfluidLevelHigh);
	//     
	//      Serial.print("Latitude : ");
	//      float platitude = (myData.latitude);      
	//      Serial.println(platitude);
	//      
	//      Serial.print("Longitude : "); 
	//      float plongitude = (myData.longitude);           
	//      Serial.println(plongitude); 
	//           
	//      Serial.print("Proximity count : ");
	//      int pproxCount = (myData.proxCount);     
	//      Serial.println(pproxCount); 
	//       
	//      Serial.print("Battery voltage : ");
	//      float pvolts = (myData.volts);       
	//      Serial.println(pvolts); 
	//      
		}
		else
		{
		  Serial.println("No reply, is serial_reliable_datagram_server running?");
		}
	  }
	  else
		Serial.println("sendtoWait failed");
	  delay(pollingInt);
  }
  else {
	cout << "firstLoop" << endl;
	delay(10000);
	firstLoop = 0;
  }
}

