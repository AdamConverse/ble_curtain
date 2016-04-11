/*
Servo Middle = 26.
*/
#include <Servo.h> 
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

int servoPin = 3; //Pin servo is connected to
Servo servo; //Create new servo object

boolean done = false;
int forward = 36;
int backward = 16;

/**************************************************************************/
/*!
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
}

/**************************************************************************/
/*!
    This function is called whenever data arrives on the RX channel
*/
/**************************************************************************/
void rxCallback(uint8_t *buffer, uint8_t len)
{
  if((char)buffer[0] == 'o')
  {
     Serial.println(F("Opening...."));
     fullSpinBackward();
     Serial.println(F("Opened!"));
  }

  if((char)buffer[0] == 'c')
  {
     Serial.println(F("Closing...."));
     fullSpinForward();
     Serial.println(F("Closed!"));
  }
}

void fullSpinBackward()
{
  moveServo(backward);
  moveServo(backward);
  moveServo(backward);
  moveServo(backward);
}

void fullSpinForward()
{
  moveServo(forward);
  moveServo(forward);
  moveServo(forward);
  moveServo(forward);
}

void moveServo(int angle)
{
  //Moves servo quarter rotation
   servo.attach(servoPin);
   servo.write(angle); // 0 for backward rotation 180 for forward rotation
   delay(490);  
   servo.detach();  
}

/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Callback Echo demo"));

  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);
  // uart.setDeviceName("NEWNAME"); /* 7 characters max! */
  uart.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
void loop()
{
  uart.pollACI();
}
