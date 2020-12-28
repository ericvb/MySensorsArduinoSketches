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
 * Version 1.5 - Eric Van Bocxlaer - deleting node to node communication code because not using anymore LCD node
 *                                 - lowering radio baudrate to 9600 baud and set TX power to 12dBm to get more range
 * Version 1.6 - Eric Van Bocxlaer - changing the original sleep(5) in the loop (Short delay to allow buttons to properly settle) to a wait(1000)
 *                                   in order trying to increase stability (pro mini 1MHz internal needs some time to stabilize) and reliability of the radio connection.
 *                                   Drawback, the node will consume a little more power...
 *                                 - adding retries in sending the message to the gateway in an attempt to increase the delivery reliability
 *                                 - adding statistics about radio TX and TX errors 
 * 
 * DESCRIPTION
 *
 * Simple binary switch example 
 * Connect button or door/window reed switch between 
 * digitial I/O pin 3 (BUTTON_PIN below) and GND.
 * http://www.mysensors.org/build/binary
 * 
 * Used IDE configuration in menu Tools
 * Board: ATmega328
 * Clock: Internal 1MHz
 * BOD: BOD 1.8V
 * EEPROM: EEPROM retained
 * Compiler LTO: LTO disabled
 * Variant: 328P / 328PA
 * Bootloader: Yes (UART0)
 * 
 * When programming the bootloader, use
 * Programmer: Arduino as ISP (MiniCore)
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
#define MY_RFM69_SPI_SPEED (1000000ul)  // on this 1MHZ arduino node, lower the SPI speed to the same speed
#define MY_RFM69_TX_POWER_DBM (12) // set starting ATC TX power to 12dBm to get more range
#define MY_RFM69_MODEM_CONFIGURATION RFM69_FSK_BR9_6_FD19_2  // 9600 baud rate: see https://www.mysensors.org/apidocs/group__RFM69SettingGrpPub.html#gaf1455cd3427c9dc4c4564542c3dafc16

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

#define MY_INDICATION_HANDLER // we will use our own indication handler to send radio rx/ts statistics of this node

#include <MySensors.h> // sketch tested with version 2.3.2, see http://librarymanager#MySensors
#include <Vcc.h>  // this lib can be found at: https://github.com/Yveaux/Arduino_Vcc and must be manually installed

#define SENSOR_NAME "Door Sensor"
#define SENSOR_VERSION "1.6"

#define CHILD_ID 1  // Each radio node can report data for up to 254 different child sensors. You are free to choose the child id yourself. 
                    // You should avoid using child-id 255 because it is used for things like sending in battery level and other (protocol internal) node specific information.
#define BUTTON_PIN 3  // Arduino Digital I/O pin for button/reed switch
#define SLEEP_A_WEEK 604800000 // 7 days

#define MAX_TX_ATTEMPTS 5 // try to send 5 times in case of no hardware ack
#define FAILED_TX_PAUSE 500 // take 500ms pause between retries

// variables/defines used in my indication handler
static uint32_t txOK = 0;
static uint32_t txERR = 0;
#define CHILD_ID_TX_OK 20
#define CHILD_ID_TX_ERR 21
MyMessage txOKmsg(CHILD_ID_TX_OK, V_VAR1);
MyMessage txERRmsg(CHILD_ID_TX_ERR, V_VAR1);

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

  present(CHILD_ID_TX_OK, S_CUSTOM);
  present(CHILD_ID_TX_ERR, S_CUSTOM);   
}

//  Check if digital input has changed and send in new value
void loop() 
{
  // Short delay to allow buttons to properly settle
  wait(1000);
  
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
  bool sent = false;
  int failedCounter = 0;
  
  // Get the update value
  int value = digitalRead(BUTTON_PIN);

  if (value != oldValue) 
  {
    // do send retries only for the gateway
    do
    {
      // Value has changed from last transmission, send the updated value
      // pin high ==> open contacts ==> door lock not closed, so V_TRIPPED not activated
      // pin low ==> closed contacts ==> door lock closed, so V_TRIPPED activated
      sent = send(msg.set(value==HIGH ? 0 : 1)); 
      if (!sent)
      {
        sleep(FAILED_TX_PAUSE); // 500ms
        failedCounter++;
      }
    } while (!sent && failedCounter < MAX_TX_ATTEMPTS); // MAX_TX_ATTEMPTS: 5

    // send the statistics
    send(txOKmsg.set(txOK));
    send(txERRmsg.set(txERR));

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

void indication(indication_t ind)
{
  switch (ind)
  {
    case INDICATION_TX:
      txOK++;
      break;
    case INDICATION_ERR_TX:
      txERR++;
      break;
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
