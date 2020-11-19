/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.1 - Eric Van Bocxlaer - Adapt to use the RFM69HW and simple signing and encryption
 *                                 - Adding debouncer for reading the button
 * Version 1.2 - Eric Van Bocxlaer - Replacing debouncer and adding sleep mode and wake up on interrupt change
 * Version 1.3 - Eric Van Bocxlaer - Inverting logic for V_TRIPPED
 *                                   button pin high ==> open contacts ==> door lock not closed, so V_TRIPPED not activated
 *                                   button pin low ==> closed contacts ==> door lock closed, so V_TRIPPED activated
 *                                 - wake up by timer each 7 days for battery status
 *                                 - Changed sketchinfo name from 'Binary Door Sensor' to 'Door Senor' because name was double in the controller Home Assistant
 * Version 1.4 - Eric Van Bocxlaer - Adding node-to-node communication for sending also status to LCD node
 * 
 * DESCRIPTION
 *
 * Simple binary switch example 
 * Connect button or door/window reed switch between 
 * digitial I/O pin 3 (BUTTON_PIN below) and GND.
 * http://www.mysensors.org/build/binary
 */

// Enable debug prints
//#define MY_DEBUG
// Enable specific RFM69 debug prints to serial monitor
//#define MY_DEBUG_VERBOSE_RFM69

#define MY_NODE_ID 4  // set the node ID manually because a MQTT gateway will not assign automatically a node Id - this must be set before the mysensors.h call

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_868MHZ // Set your frequency here
#define MY_IS_RFM69HW // Omit if your RFM is not "H"
#define MY_RFM69_NEW_DRIVER

//enable radio communication encryption
// more information can be found on https://forum.mysensors.org/topic/10382/security-signing-messages-and-encryption-of-messages-a-guide-or-more-a-summary-of-my-tests?_=1588348189475
//#define MY_ENCRYPTION_SIMPLE_PASSWD "your16bitpassword"
//enable simple signing
//#define MY_SIGNING_SIMPLE_PASSWD "your32bitpassword"
//#define MY_SIGNING_SIMPLE_PASSWD "your16bitpassword"
//enable simple signing and encryption
//#define MY_SECURITY_SIMPLE_PASSWORD "your16bitpassword"
//enable simple signing and encryption
#define MY_SECURITY_SIMPLE_PASSWORD "your32bitpassword"
//enable soft signing
//#define MY_SIGNING_SOFT
//#define MY_SIGNING_REQUEST_SIGNATURES
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN A0 
// following hex codes are dummy hex codes, replace by your hexcodes (see the link above how to generate)
//#define MY_SIGNING_NODE_WHITELISTING {{.nodeId = 0,.serial = {0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11}},{.nodeId = 1,.serial = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99}}}

#include <MySensors.h> // sketch tested with version 2.3.2, see http://librarymanager#MySensors
#include <Vcc.h>  // this lib can be found at: https://github.com/Yveaux/Arduino_Vcc and must be manually installed

#define SENSOR_NAME "Door Sensor"
#define SENSOR_VERSION "1.4"

/*
 * Send the message not only to the gateway, but use also node-to-node communication
 */
#define LCD_NODE_ID 100  // node id of the LCD sensor node
#define LCD_CHILD_ID 3   //destination virtual child sensor of the LCD  node: used by the LCD node to know what to do on the LCD screen

#define CHILD_ID 1  // Each radio node can report data for up to 254 different child sensors. You are free to choose the child id yourself. 
                    // You should avoid using child-id 255 because it is used for things like sending in battery level and other (protocol internal) node specific information.
#define BUTTON_PIN 3  // Arduino Digital I/O pin for button/reed switch
#define SLEEP_A_WEEK 604800000 // 7 days

const float VccMin        = 2.0*0.6;  // Minimum expected Vcc level, in Volts. Example for 2xAA Alkaline.
const float VccMax        = 2.0*1.5;  // Maximum expected Vcc level, in Volts. Example for 2xAA Alkaline.
const float VccCorrection = 1.0/1.0;  // Measured Vcc by multimeter divided by reported Vcc

Vcc vcc(VccCorrection);
static int oldBatteryPcnt = 0;

int oldValue=-1;

// Change to V_LIGHT if you use S_LIGHT in presentation below
// V_TRIPPED : on ==> activated
// V_TRIPPED : off ==> not activated
MyMessage msg(CHILD_ID,V_TRIPPED);
MyMessage msgLCD(LCD_CHILD_ID, V_STATUS);

void setup()  
{  
  setUnusedPins();  
  // We will not use the internal pullup resistance, but a higher external pullup resistance of 1M ohm.
  pinMode(BUTTON_PIN,INPUT);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SENSOR_NAME, SENSOR_VERSION);
    
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  present(CHILD_ID, S_DOOR);  
}

//  Check if digital input has changed and send in new value
void loop() 
{
  // Short delay to allow buttons to properly settle
  sleep(5);
  
  getAndSendSensorValue();

  getAndSendBatteryLevel(); 

  // Sleep until something happens with the sensor
  // interrupt - Interrupt that should trigger the wakeup.(Pro Mini: 0=Pin2, 1=Pin3)
  // mode - RISING, FALLING, CHANGE
  // ms - Number of milliseconds to sleep (or 0 to sleep forever): in case the door isn't unlocked or locked for a long time
  // wakeup once a week and do the check of battery!
  sleep(BUTTON_PIN-2, CHANGE, SLEEP_A_WEEK);  
}

void getAndSendSensorValue()
{
  // Get the update value
  int value = digitalRead(BUTTON_PIN);

  if (value != oldValue) 
  {
     // Value has changed from last transmission, send the updated value
     // pin high ==> open contacts ==> door lock not closed, so V_TRIPPED not activated
     // pin low ==> closed contacts ==> door lock closed, so V_TRIPPED activated
     send(msg.set(value==HIGH ? 0 : 1)); 
     send(msgLCD.setDestination(LCD_NODE_ID).set(value==HIGH ? 0 : 1)); // send also to LCD node
     oldValue = value;
  }  
}

void getAndSendBatteryLevel()
{
  int batteryPcnt = (int)vcc.Read_Perc(VccMin, VccMax);
  if (oldBatteryPcnt != batteryPcnt)
  {
    sendBatteryLevel(batteryPcnt);
    oldBatteryPcnt = batteryPcnt;
  }
}

/*
 * https://forum.arduino.cc/index.php?topic=104073.0
 * If you leave unused pins unconnected and unprogrammed, they default to (input, low). 
 * In this state the pin may float at a voltage that is neither high nor low, which slightly increases the current consumption 
 * of the input buffer. Unless you're using the sleep mode of the processor, you probably won't notice the difference. 
 * Programming unused pins as (input, high) to enable the internal pullup avoids the issue. So does programming them as outputs.
 * 
 * In battery powered environments, unused pins not set, can consume extra power.
 */
void setUnusedPins()
{
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);
  pinMode(A3,INPUT_PULLUP);
  pinMode(A4,INPUT_PULLUP);
  pinMode(A5,INPUT_PULLUP);
  pinMode(A6,INPUT_PULLUP);
  pinMode(A7,INPUT_PULLUP);
}
