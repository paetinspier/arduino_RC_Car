#include <Adafruit_BLE_UART.h>
#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

//#define AIN1 3
//#define AIN2 4
//#define BIN1 6
//#define BIN2 7
#include <Stepper.h>

// MOTOR PURPOSES
#define Ain1 3
#define Ain2 4
#define Bin1 6
#define Bin2 5
int speed = 0;

// ultrasonic sensor (pin numbers)
const int trigPin = 7; // change later
const int echoPin = 8;

// defines variables
//long duration;
//int distance

// from echoDemo defines
/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/

void setup() {
  
// input
pinMode(1, OUTPUT);

// for the ultrasonic sensor 
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

// FOR MOTOR PURPOSES
pinMode(Ain1, OUTPUT);  //Ain1
pinMode(Ain2, OUTPUT);  //Ain2
pinMode(Bin1, OUTPUT);  //Bin1
pinMode(Bin2, OUTPUT);  //Bin2

//from voidsetup (void) echodemo class
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("NEWNAME"); /* 7 characters max! */

  BTLEserial.begin();

}
// echo demo void setup
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {

  // for the ultrasonic sensor
  // Clears the trigPin
  long duration;
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

// FOR MOTOR PURPOSES
// all on makes motor spin forward
//********************************
    //digitalWrite(Ain1,LOW); 
    //digitalWrite(Ain2,HIGH);
    //digitalWrite(Bin2,LOW);  
    //digitalWrite(Bin1,HIGH);
    
//********************************

//********************************
    //delay(700);
    //digitalWrite(Ain1,LOW);
    //digitalWrite(Ain2,LOW);
    //digitalWrite(Bin1,LOW);
    //digitalWrite(Bin2,LOW);
    //digitalWrite(Ain1,LOW);
    //digitalWrite(Ain2,HIGH);
    //digitalWrite(Bin1,LOW);
    //digitalWrite(Bin2,HIGH);
    //delay(700);
    //digitalWrite(Ain1,LOW);
    //digitalWrite(Ain2,LOW);
    //digitalWrite(Bin1,LOW);
    //digitalWrite(Bin2,LOW);


  // from echodemo void loop()
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      //Serial.println("The button pressed = "); 
      //Serial.println("printing serial number");
      //Serial.print(BTLEserial.available());
      //Serial.println("end of serial number");
      //Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      String c = BTLEserial.readString();
      Serial.println(c);

      if (c == "!B516"){ //forward
        digitalWrite(Ain1,LOW); 
        digitalWrite(Ain2,HIGH);
        digitalWrite(Bin2,LOW);  
        digitalWrite(Bin1,HIGH);
      }
      else if (c == "!B615"){ //backwards
        digitalWrite(Ain1,HIGH); 
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,HIGH);  
        digitalWrite(Bin1,LOW);
      }
       else if (c == "!B714"){ //LEFT
        digitalWrite(Ain1,LOW); 
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,LOW);  
        digitalWrite(Bin1,HIGH);
      }
       else if (c == "!B813"){ //RIGHT
        digitalWrite(Ain1,LOW); 
        digitalWrite(Ain2,HIGH);
        digitalWrite(Bin2,LOW);  
        digitalWrite(Bin1,LOW);
      }
      else if (c == "!B11:"){ //spin clockwise
        digitalWrite(Ain1,LOW); 
        digitalWrite(Ain2,HIGH);
        digitalWrite(Bin2,HIGH);  
        digitalWrite(Bin1,LOW);
      }
      else if (c == "!B219"){ //spin counter clockwise
        digitalWrite(Ain1,HIGH); 
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,LOW);  
        digitalWrite(Bin1,HIGH);
      }
      else if (c == "!B507" || c == "!B606" || c == "!B705" || c == "!B804" || c == "!B10;" || c == "!B20:" || c == "!B309" || c == "!B408"){ //STOP
        digitalWrite(Ain1,LOW); 
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,LOW);  
        digitalWrite(Bin1,LOW);
      }

      }
 
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(500); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); 
      Serial.print((char *)sendbuffer); 
      Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  
}   
