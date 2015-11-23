/*
 * File: _end_node_9_debug.ino
 * Date: 11/222015
 * Author: DJ 
 * Desc: Program for Arduino-based (nano, pro mini, etc.) end-node for sensor data collection and driving relays and motors
 * Functions included:
 *    - RF69_reliable_datagram_server, RF69 is 3.3V device, uses level shift for 5V MCU
 *    - includes encryption
 *    - DHT11 sensor (air temp and air humidity): pin A5 or digital 19 (using it as digital)
 *    - Soil humidity sensor: pin A0 for analog measurement, and pin D4 for turning on/off (duty cycle to prevent rust and reduce power consumption)
 *    - Rain sensor: pin A1 for analog measurement, and pin D5 for turning on/off (duty cycle to prevent rust and reduce power consumption)
 *    - Fluid level: abstracting it as an On/Off switch, using pin D6 with internal pull-up R (switch pressed produces LOW on pin)
 *    - GPS sensor: it's 3.3V device, uses level shift for 5V MCU, gps-RX->L6 gps-TX->L7 and H6->D7(TX pin) H7->D8(RX pin), using SoftSerialLib for D7/D8 as TX/RX
 *    - Sharp IR proximity sensor (not analog - the analog requires A pin): pin D3 (requires interrupt, D3=INT1), can be used for PIR, but need to reverse client/server
 *    - voltage sensor for internal battery: pin A2 (analog 10-bit measurement on the scale 0-25V, resolution 
 *    - relay 1 and 2 outputs: pin 17 and 18 (A3 and A4) - used as digital pins
 *    - transistor driver output as motor speed regulator (MOSFET on PWM pin): pin D9
 *    
 * This debug version is for end-node 1
 */

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#include "DHT.h"  //for DHT11 sensor

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define CLIENT_ADDRESS 1  // gateway address
#define SERVER_ADDRESS1 2  // end-node 1: this is server,it's responding on the request from the client (gateway)
#define SERVER_ADDRESS2 3  // end-node 2
#define SERVER_ADDRESS3 4  // end-node 3

#define DHTPIN 19
#define DHTTYPE DHT11  //for DHT11 sensor

// Singleton instance of the radio driver
RH_RF69 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS1);

DHT dht(DHTPIN, DHTTYPE);

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

byte tx_buf[sizeof(myData)] = {0};

int soilHumidSensor = 0;  // definition of soil humidity pin: A0
int soilHumidOn = 4; // definition of soil humidity sensor power on/off: D4

int rainSensor = 1;  // definition of rain sensor pin: A1
int rainSensorOn = 5; // definition of rain sensor power on/off: D5

int fluidLevel = 6; // definition of the fluid level sensor pin: D6

static const int RXPin = 8, TXPin = 7; // definition of GPS pins: D8 and D7
static const uint32_t GPSBaud = 4800;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

int sharpIR = 3; // definition of the proximity sensor pin: D3 (needs interrupt)
int i = 0;       // counter to store the number of proximity hits (reset after each poll to reduce mem reqs

int battSensor = 2; // definition of the voltage sensor pin: A2

int relay1 = 17; // definition for relay 1 pin: A3, but used as digital pin, so using D number
int relay2 = 18; // definition for relay 2 pin: A4, but used as digital pin, so using D number

int mosfet = 9; // definition for MOSFET pin: 9 (D9 is PWM capable)

void setup() 
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  
  if (!manager.init())
    Serial.println(F("init failed"));

  // If you are using a high power RF69, you *must* set a Tx power in the
  // range 14 to 20 like this:
  driver.setTxPower(14);
  
   // The encryption key has to be the same as the one in the client
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  driver.setEncryptionKey(key);

  #if 0
  // For compat with RFM69 Struct_send
  driver.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  driver.setPreambleLength(3);
  uint8_t syncwords[] = { 0x2d, 0x64 };
  driver.setSyncWords(syncwords, sizeof(syncwords));
  driver.setEncryptionKey((uint8_t*)"thisIsEncryptKey");
  #endif

  dht.begin();

  pinMode(soilHumidOn, OUTPUT);
  pinMode(rainSensorOn, OUTPUT);

  pinMode(fluidLevel, INPUT_PULLUP);

  pinMode(sharpIR, INPUT);
  attachInterrupt(1, increment, FALLING);

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, HIGH); // need to check if relay will be mostly on or off - potentially invert the logic
  digitalWrite(relay2, HIGH); // same as above

  pinMode(mosfet, OUTPUT);
}

uint8_t data[] = "Values";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop()
{
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
//    Receiving instructions and poll command from the gateway
      Serial.print(F("got request from : 0x"));
      Serial.print(from, HEX);
      Serial.print(F(": "));
      String input = ((char*)buf);
      Serial.println((char*)buf);
      int nodeIdent = (input.substring(0,1)).toInt();
      int push_relay1 = (input.substring(1,2)).toInt();
      int push_relay2 = (input.substring(2,3)).toInt();
      int push_pwm = (input.substring(3,7)).toInt();

      if (nodeIdent == SERVER_ADDRESS1){
        Serial.print(F("nodeID: "));
        Serial.println(nodeIdent);
        myData.nodeID = nodeIdent; // confirmation back to rpi
  
  //    Collecting all the sensor inputs from the end-node
        myData.temp = dht.readTemperature(true);
        Serial.print(F("temp: "));
        Serial.println(myData.temp);
        
        myData.airHumid = dht.readHumidity();
        Serial.print(F("air humidity: "));
        Serial.println(myData.airHumid);
  
        myData.soilHumidPerc = analogValueSub(soilHumidSensor,soilHumidOn);
        Serial.print(F("soil humidity: "));
        Serial.println(myData.soilHumidPerc);
  
        myData.rainPerc = analogValueSub(rainSensor, rainSensorOn);
        Serial.print(F("rain percentage: "));
        Serial.println(myData.rainPerc);
  
        myData.fluidLevelHigh = (digitalRead(fluidLevel) == LOW) ? true : false;
        Serial.print(F("fluid level: "));
        Serial.println(myData.fluidLevelHigh);
  
        gpsSub();
        Serial.print(F("latitude: "));
        Serial.println(myData.latitude); 
        Serial.print(F("longitude: "));
        Serial.println(myData.longitude);
            
        myData.proxCount = i;
        Serial.print(F("Proximity detections: "));
        Serial.println(myData.proxCount);
        
        myData.volts = battVolts();
        Serial.print(F("Battery voltage: "));
        Serial.println(myData.volts);
  
        int buffSize = sizeof(myData);
        Serial.print(F("buffSize: "));
        Serial.println(buffSize);
  
  //    Applying commands received from the gateway
        Serial.print(F("relay1 cmd: "));
        Serial.println(push_relay1);
        if(push_relay1 == 1){
          digitalWrite(relay1, LOW);
        }
        else {
          digitalWrite(relay1, HIGH);
        }
  
        Serial.print(F("relay2 cmd: "));
        Serial.println(push_relay2);
        if(push_relay2 == 1){
          digitalWrite(relay2, LOW);
        }
        else {
          digitalWrite(relay2, HIGH);
        }
  
        Serial.print(F("pwm cmd: "));
        Serial.println(push_pwm);
        analogWrite(mosfet, push_pwm);
               
  //    Sending information back to gateway
        memcpy(tx_buf, &myData, sizeof(myData) );     
        
        // Send a reply back to the originator client
        if (!manager.sendtoWait(tx_buf, sizeof(myData), from))
          Serial.println(F("sendtoWait failed")); 
  
  
        i = 0; // reset proximity hit counter at the end of the loop (after previous count has been sent)
      }
      else {
        Serial.println(F("RPi nodeIdent and SERVERADDRESS do not match"));
      }
    }
  }
}

int analogValueSub(int sensorDef, int turnOnPin){
  digitalWrite(turnOnPin, HIGH);
  delay(10); // try to reduce this value, how long it takes for power to stabilize, check with DSO??
  int sVal = analogRead(sensorDef)*0.09765625; // to get sVal in %, the float is 100/1024 - as analog input is 10bit or 2^10
  digitalWrite(turnOnPin, LOW);
  return sVal;
}

void gpsSub(){
  if (ss.available() > 0){
    if (gps.encode(ss.read())){
      if (gps.location.isValid()){
        myData.latitude = (gps.location.lat(), 6);
        myData.longitude = (gps.location.lng(), 6);
        // possible to get the time to centisecond from the satellite, future option to mark each record stream from the end-node
      }
    }
  }
}

void increment() {
  i++;
}

float battVolts(){
  return analogRead(battSensor)/1024.00*25; // 25 is the max voltage measured on voltage sensor (R-divider)
}

