/*
 * File: _gw_9_debug.ino
 * Date: 11/22/2015
 * Author: DJ 
 * Desc: Program for Arduino-based (nano, pro mini, etc.) gateway for communication with end-nodes
 * Functions included:
 *    - RF69_reliable_datagram_server, RF69 is 3.3V device, uses level shift for 5V MCU
 *    - includes encryption
 *    - includes LCD 16x2
 *    - communication to RPi through FTDI (USB0 on Rpi) - MCU resets on initialization of USB, keep it running
 *    
 * This debug version is for gateway
 */

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <RH_Serial.h>
#include <SPI.h>

#include <SoftwareSerial.h>

#define CLIENT_ADDRESS 1  // this is client for wireless communication to end-nodes
#define SERVER_ADDRESS1 2
#define SERVER_ADDRESS2 3
#define SERVER_ADDRESS3 4

#define SERIAL_CLIENT_ADDRESS 251
#define SERIAL_SERVER_ADDRESS 252  // this is server for serial communication to RPi

// Singleton instance of the radio driver
RH_RF69 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

SoftwareSerial lcd(4, 5);  // This is required, to start an instance of an LCD

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

byte tx_buf[sizeof(myData)] = {0}; // use this to send to Serial

 RH_Serial serial_driver(Serial);
 RHReliableDatagram serial_manager(serial_driver, SERIAL_SERVER_ADDRESS);

void setup()
{
  Serial.begin(9600);
  if (!manager.init())
    Serial.println(F("init failed"));

  // If you are using a high power RF69, you *must* set a Tx power in the
  // range 14 to 20 like this:
  driver.setTxPower(14);
  
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  driver.setEncryptionKey(key);
  

  lcd.begin(9600);  // Start the LCD at 9600 baud
  clearDisplay();  // Clear the display
  setLCDCursor(0);  // Set cursor to the 3rd spot, 1st line
  lcd.print("Init in progress!");
  setLCDCursor(16);  // Set the cursor to the beginning of the 2nd line
  lcd.print("Please wait...");

  // Flash the backlight:
  for (int i = 0; i<3; i++)
  {
    setBacklight(0);
    delay(250);
    setBacklight(255);
    delay(250);
  }

  serial_driver.serial().begin(9600);
  if (!serial_manager.init())
    Serial.println(F("serial init failed"));
    
}

uint8_t data[] = "20000001";  // make this dataStruct to send commands to end-nodes
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

uint8_t serial_data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t serial_buf[RH_SERIAL_MAX_MESSAGE_LEN];

void loop()
{
  serial_manager.waitAvailable();
  uint8_t serial_len = sizeof(serial_buf);
  uint8_t serial_from;
  if (serial_manager.recvfromAck(serial_buf, &serial_len, &serial_from)){  // all this is getting the data from rpi and parsing command instructions
      Serial.print(F("got request from : 0x"));
      Serial.print(serial_from, HEX);
      Serial.print(F(": "));
      String input = ((char*)serial_buf);
      Serial.println((char*)serial_buf);
      int nodeIdent = (input.substring(0,1)).toInt();
      int push_relay1 = (input.substring(1,2)).toInt();
      int push_relay2 = (input.substring(2,3)).toInt();
      int push_pwm = (input.substring(3,7)).toInt();

      input.getBytes(data, input.length()+1); // sending the instruction received from rpi to endnode
      
    Serial.println(F("Sending to rf69_server"));
    
    // Send a message to manager_server
//    if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
    if (manager.sendtoWait(data, sizeof(data), nodeIdent))  // sending the data received from rpi to end-node requested from rpi
    {
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
      {
        Serial.print(F("got reply from : 0x"));
        Serial.print(from, HEX);
        Serial.print(F(": "));
  
          // Message with a good checksum received, dump it.
  //    driver.printBuffer("Got:", buf, &len);
        memcpy(&myData, buf, sizeof(myData));
        
        Serial.println(myData.nodeID);
        Serial.println(myData.temp);
        Serial.println(myData.airHumid);
        Serial.println(myData.soilHumidPerc);      
        Serial.println(myData.rainPerc);      
        Serial.println(myData.fluidLevelHigh);
        Serial.println(myData.latitude);
        Serial.println(myData.longitude);      
        Serial.println(myData.proxCount); 
        Serial.println(myData.volts);       
               
        clearDisplay();  // Clear the display
        setLCDCursor(1);  // set cursor to 2nd spot, 1st row
        lcd.print(F("Temp : "));
        lcd.print(myData.temp);  // print the temperature
  
        setLCDCursor(17);  // set the cursor to the 5th spot, 2nd row
        lcd.print(F("H/Cnt: "));
        lcd.print(myData.airHumid);  // print the count
        lcd.print(F("/"));
        lcd.print(myData.proxCount);  // print the total count
  
      }
      else
      {
        Serial.println(F("No reply, is rf69_reliable_datagram_server running?"));
      }
    }
    else
      Serial.println(F("sendtoWait failed"));
//    delay(5000);


//    Sending information back to RPi through serial
      memcpy(tx_buf, &myData, sizeof(myData) ); 
      
// Send a reply back to the serial client
//    if (!serial_manager.sendtoWait(serial_data, sizeof(serial_data), serial_from))
    if (!serial_manager.sendtoWait(tx_buf, sizeof(myData), serial_from))
      Serial.println(F("serial sendtoWait failed"));
          ;
  }
}

void setBacklight(byte brightness)
{
  lcd.write(0x80);  // send the backlight command
  lcd.write(brightness);  // send the brightness value
}

void clearDisplay()
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x01);  // send the clear screen command
}

void setLCDCursor(byte cursor_position)
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x80);  // send the set cursor command
  lcd.write(cursor_position);  // send the cursor position
}


