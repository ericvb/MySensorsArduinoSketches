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
 *                                 - Adding support for Nextion display
 *                                 - Adding direct node to node communication for sending directly their status to this LCD node
 *                                 - As backup this LCD node will also request the controller to send the status of the other nodes. Used for example when booting the LCD node.
 *                                 
 * NEXTION Display
 * After installation of the library (see furter <Nextion.h>) you will find documentation in : 
 * file:///C:/Users/xxx/Documents/Arduino/libraries/ITEADLIB_Arduino_Nextion/doc/Documentation/index.html
 * 
 * DESCRIPTION
 * Display node: receives his commands from the mysensors network
 */


// Enable debug prints to serial monitor. Pay attention a pro mini for example has only one hardware serial port, so a nextion display and the serial
// monitor at the same time will not work
//#define MY_DEBUG 
// Enable specific RFM69 debug prints to serial monitor
//#define MY_DEBUG_VERBOSE_RFM69

#define MY_NODE_ID 100  // set the node ID manually because a MQTT gateway will not assign automatically a node Id - this must be set before the mysensors.h call

#define MY_REPEATER_FEATURE  // define this node as a repeater. Comment it if you don't need it as repeater.

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_868MHZ // Set your frequency here
#define MY_IS_RFM69HW // Omit if your RFM is not "H"
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_TX_POWER_DBM (12) // set starting ATC TX power directly to 12dBm to have more range. Not really needed as ATC will increase automatically.
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

#include <MySensors.h>
#include <Nextion.h> // sketch tested with version 0.9.0, see https://github.com/itead/ITEADLIB_Arduino_Nextion : download master branch as zip and import zip in the Arduino library
#include <TimeLib.h> // sketch tested with version 1.6, see http://librarymanager#Time from Michael Margolis

#define SENSOR_NAME "LCD Display Panel Sensor"
#define SENSOR_VERSION "1.1"

// see also the HMI editor file of the Nextion display
#define NEXTION_PICTURE_ID_GREEN_SMILEY 1
#define NEXTION_PICTURE_ID_RED_SMILEY 2
#define NEXTION_PICTURE_ID_NETWORK 3
#define NEXTION_PICTURE_ID_NO_NETWORK 4

// This LCD node will get his values directly by node to node communication. Each sending node has a specific same define --> #define LCD_CHILD_ID_pxxxxxx
// Besides the direct node to node communication, this LCD node will also set and get his values from the controller because sometimes with node to node communication, messages are lost...
// and at startup it must poll the controller to get the current statusses of the nodes
// It will do this by creating with the same #define LCD_CHILD_ID_pxxxxxxxxx, V_CUSTOM fields in the controller.
// The controller Home Assistant will copy the states of the other nodes into these fields.
// REMARK:
// The controller HA has no direct possibility to copy statusses from one node to another, but as workaround we can use Automations who can generate fake mqtt messages
// simulating that this LCD nodes is sending updates to the controller for his V_CUSTOM fields
#define LCD_CHILD_ID_pGarBike 1
#define LCD_CHILD_ID_pGarCar 2
#define LCD_CHILD_ID_pDoorHobby 3
#define LCD_CHILD_ID_pDoorStorage 4
#define LCD_CHILD_ID_pDoorVeranda 5
#define LCD_CHILD_ID_pDoorFrontdoor 6

// declare the nextion objects
NexPicture statusGarBike = NexPicture(0,3,"pGarFiets");
NexPicture statusGarCar = NexPicture(0,4,"pGarAuto");
NexPicture statusDoorHobby = NexPicture(0,7,"pDoorTrainkot");
NexPicture statusDoorStorage = NexPicture(0,11,"pDoorBerging");
NexPicture statusDoorVeranda = NexPicture(0,12,"pDoorVeranda");
NexPicture statusDoorFrontdoor = NexPicture(0,13,"pDoorVoordeur");

NexPicture statusRadioNetwork = NexPicture(0,26,"pRadio");

bool isGarageBikeClosed = true;
bool isGarageCarClosed = true;
bool isDoorStorageClosed = true;
bool isDoorHobbyClosed = true;
bool isDoorFrontdoorClosed = true;
bool isDoorVerandaClosed = true;

bool timeReceived = false;
bool isThspSetTo60 = true;  // set in the setup()
bool receivedRealMessage = false;
int receivedLCDChildId;

unsigned long lastTimeRequest=0;  // request the controller time
unsigned long REQUEST_TIME = 60000*1; //every 5 minutes.
unsigned long lastRequestTime=0;
unsigned long HEARTBEAT_TIME = 60000*60; //every hour.
unsigned long lastHeartbeatTime = 0;

void setup()  
{   
  // You might need to change NexConfig.h file in your ITEADLIB_Arduino_Nextion folder
  // Set the baudrate which is for debug and communicate with Nextion screen
  nexInit();  // default 9600 baud

  sendCommand("thup=1"); // auto wake on touch
  sendCommand("thsp=60"); // auto sleep after 1 minute no touch
  
  // Request time from controller. 
  requestTime(); 
  
  // Fetch last known statusses from controller
  request(LCD_CHILD_ID_pGarBike, V_CUSTOM);
  wait(500); // provide time for the node to receive the answer   
  request(LCD_CHILD_ID_pGarCar, V_CUSTOM);
  wait(500);
  request(LCD_CHILD_ID_pDoorHobby, V_CUSTOM);
  wait(500);
  request(LCD_CHILD_ID_pDoorStorage, V_CUSTOM);
  wait(500);
  request(LCD_CHILD_ID_pDoorVeranda, V_CUSTOM);
  wait(500);
  request(LCD_CHILD_ID_pDoorFrontdoor, V_CUSTOM);
  wait(500);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SENSOR_NAME, SENSOR_VERSION);

  // Register all sensors to gw (they will be created as child devices) by their ID and S_TYPE
  // for Home Assistant : S_CUSTOM together with V_CUSTOM
  present(LCD_CHILD_ID_pGarBike, S_CUSTOM); 
  present(LCD_CHILD_ID_pGarCar, S_CUSTOM); 
  present(LCD_CHILD_ID_pDoorHobby, S_CUSTOM); 
  present(LCD_CHILD_ID_pDoorStorage, S_CUSTOM); 
  present(LCD_CHILD_ID_pDoorVeranda, S_CUSTOM); 
  present(LCD_CHILD_ID_pDoorFrontdoor, S_CUSTOM); 
}

void loop() 
{
  // uncomment following code to create the very first time the variables V_CUSTOM on the Home Assistant controller side
  // program your node and let it run.
  // after creating the entries in the mysensors.json file on the HA side, comment again these child sensors
//  send(MyMessage(LCD_CHILD_ID_pGarBike,V_CUSTOM).set(1));
//  send(MyMessage(LCD_CHILD_ID_pGarCar,V_CUSTOM).set(1));
//  send(MyMessage(LCD_CHILD_ID_pDoorHobby,V_CUSTOM).set(1));
//  send(MyMessage(LCD_CHILD_ID_pDoorStorage,V_CUSTOM).set(1));
//  send(MyMessage(LCD_CHILD_ID_pDoorVeranda,V_CUSTOM).set(1));
//  send(MyMessage(LCD_CHILD_ID_pDoorFrontdoor,V_CUSTOM).set(1));
  
  unsigned long now = millis();

  // If no time has been received yet, request it every 10 second from controller
  // When time has been received, request update every hour
  if ((!timeReceived && (now-lastTimeRequest) > (10UL*1000UL))
    || (timeReceived && (now-lastTimeRequest) > (60UL*1000UL*60UL))) 
  {
    // Request time from controller. 
    #ifdef MY_DEBUG
    Serial.println("requesting time");
    #endif    
    
    requestTime();  
    lastTimeRequest = now;
  }  

  sendHeartbeatToController(); //each day a check 
  wait(1000); // wait 1 seconds and process incoming data as repeater node
  
  RequestNodesStatusFromController();  // poll the controller and request the V_CUSTOM fields
  wait(1000); // wait 1 seconds and process incoming data as repeater node
  
  setDisplaySleepStatus();
  wait(1000); // wait 1 seconds and process incoming data as repeater node
  
  SendNodesStatusToController();  // in case we did get a direct node to node communication, set the V_CUSTOM field on the controller

  wait(10000); // wait 10 seconds and process incoming data as repeater node
}

void receive(const MyMessage &message) 
{
  #ifdef MY_DEBUG
  Serial.println("**********************************Entering receive method of MySensors...**************************************");
  Serial.print("Message type is: ");
  Serial.println(message.type);
  Serial.print("Message from node: ");
  Serial.println(message.sender);
  Serial.print("Message sensorId is: ");
  Serial.println(message.sensor);
  Serial.print("Message value is (getBool()): ");
  Serial.println(message.getBool());
  Serial.print("Message value is (getInt()): ");
  Serial.println(message.getInt());
  Serial.print("Message value is (getString()): ");
  Serial.println(message.getString());
  #endif
  // we are receiving radio messages
  statusRadioNetwork.Set_background_image_pic(NEXTION_PICTURE_ID_NETWORK);

  // set only if receiving real node messages.
  // boolean receivedRealMessage used to update the controller child sensors of this node
  if (message.type == V_STATUS)
  {      
    receivedRealMessage = true;
  }
  receivedLCDChildId = message.sensor;
    
  if (message.type == V_STATUS || message.type == V_CUSTOM) 
  {
    #ifdef MY_DEBUG
    Serial.println("Entering message.type == V_STATUS || message.type == V_CUSTOM...");
    Serial.print("Message type is: ");
    Serial.println(message.type);
    #endif

    bool isClosed = false;

    // message.getInt() is used instead of the message.getBool() because by mistake the sending nodes are sending 0 or 1 as integer and not a boolean
    if (message.getInt() == 1)
    {
      isClosed = true;
    }

    #ifdef MY_DEBUG
    Serial.println("Status of isClosed...");
    Serial.print("IsClosed: ");
    Serial.println(isClosed);
    #endif
  
    switch (message.sensor) {
      case LCD_CHILD_ID_pGarBike:      //incoming message is for pGarBike
        isClosed ? statusGarBike.Set_background_image_pic(NEXTION_PICTURE_ID_GREEN_SMILEY)
                 : statusGarBike.Set_background_image_pic(NEXTION_PICTURE_ID_RED_SMILEY);
        isGarageBikeClosed = isClosed;
        break;
      case LCD_CHILD_ID_pGarCar:       //incoming message is for pGarCar
        isClosed ? statusGarCar.Set_background_image_pic(NEXTION_PICTURE_ID_GREEN_SMILEY)
                 : statusGarCar.Set_background_image_pic(NEXTION_PICTURE_ID_RED_SMILEY);
        isGarageCarClosed = isClosed;
        break;
      case LCD_CHILD_ID_pDoorHobby:  //incoming message is for pDoorHobby
        isClosed ? statusDoorHobby.Set_background_image_pic(NEXTION_PICTURE_ID_GREEN_SMILEY)
                 : statusDoorHobby.Set_background_image_pic(NEXTION_PICTURE_ID_RED_SMILEY);
        isDoorHobbyClosed = isClosed;                                 
        break;
      case LCD_CHILD_ID_pDoorStorage:   //incoming message is for pDoorStorage
        isClosed ? statusDoorStorage.Set_background_image_pic(NEXTION_PICTURE_ID_GREEN_SMILEY)
                 : statusDoorStorage.Set_background_image_pic(NEXTION_PICTURE_ID_RED_SMILEY);
        isDoorStorageClosed = isClosed;                                 
        break;
      case LCD_CHILD_ID_pDoorVeranda:   //incoming message is for pDoorVeranda
        isClosed ? statusDoorVeranda.Set_background_image_pic(NEXTION_PICTURE_ID_GREEN_SMILEY)
                 : statusDoorVeranda.Set_background_image_pic(NEXTION_PICTURE_ID_RED_SMILEY);
        isDoorVerandaClosed = isClosed;                                 
        break;
      case LCD_CHILD_ID_pDoorFrontdoor:  //incoming message is for pDoorFrontdoor
        isClosed ? statusDoorFrontdoor.Set_background_image_pic(NEXTION_PICTURE_ID_GREEN_SMILEY)
                 : statusDoorFrontdoor.Set_background_image_pic(NEXTION_PICTURE_ID_RED_SMILEY);
        isDoorFrontdoorClosed = isClosed;                                 
        break;
    }      

    #ifdef MY_DEBUG
    Serial.println("Status of sensors...");
    Serial.print("isGarageBikeClosed: ");
    Serial.println(isGarageBikeClosed);
    Serial.print("isGarageCarClosed: ");
    Serial.println(isGarageCarClosed);
    Serial.print("isDoorHobbyClosed: ");
    Serial.println(isDoorHobbyClosed);
    Serial.print("isDoorStorageClosed: ");
    Serial.println(isDoorStorageClosed);
    Serial.print("isDoorVerandaClosed: ");
    Serial.println(isDoorVerandaClosed);
    Serial.print("isDoorFrontdoorClosed: ");
    Serial.println(isDoorFrontdoorClosed);
    #endif
  
    setDisplaySleepStatus();
  }
  Serial.println("***********************************************************************************************************************");
}

void SendNodesStatusToController()
{
  if (receivedRealMessage)
  {
    #ifdef MY_DEBUG
    Serial.println("Sending V_CUSTOM messages...");
    Serial.println("Status of sensors...");
    Serial.print("isGarageBikeClosed: ");
    Serial.println(isGarageBikeClosed);
    Serial.print("isGarageCarClosed: ");
    Serial.println(isGarageCarClosed);
    Serial.print("isDoorHobbyClosed: ");
    Serial.println(isDoorHobbyClosed);
    Serial.print("isDoorStorageClosed: ");
    Serial.println(isDoorStorageClosed);
    Serial.print("isDoorVerandaClosed: ");
    Serial.println(isDoorVerandaClosed);
    Serial.print("isDoorFrontdoorClosed: ");
    Serial.println(isDoorFrontdoorClosed);    
    #endif  

    switch (receivedLCDChildId) {
      case LCD_CHILD_ID_pGarBike:      //incoming message is for pGarBike
        send(MyMessage(LCD_CHILD_ID_pGarBike,V_CUSTOM).set(isGarageBikeClosed ? 1 : 0));
        break;
      case LCD_CHILD_ID_pGarCar:       //incoming message is for pGarCar
        send(MyMessage(LCD_CHILD_ID_pGarCar,V_CUSTOM).set(isGarageCarClosed ? 1 : 0));
        break;
      case LCD_CHILD_ID_pDoorHobby:  //incoming message is for pDoorHobby
        send(MyMessage(LCD_CHILD_ID_pDoorHobby,V_CUSTOM).set(isDoorHobbyClosed ? 1 : 0));
        break;
      case LCD_CHILD_ID_pDoorStorage:   //incoming message is for pDoorStorage
        send(MyMessage(LCD_CHILD_ID_pDoorStorage,V_CUSTOM).set(isDoorStorageClosed ? 1 : 0));
        break;
      case LCD_CHILD_ID_pDoorVeranda:   //incoming message is for pDoorVeranda
        send(MyMessage(LCD_CHILD_ID_pDoorVeranda,V_CUSTOM).set(isDoorVerandaClosed ? 1 : 0));
        break;
      case LCD_CHILD_ID_pDoorFrontdoor:  //incoming message is for pDoorFrontdoor
        send(MyMessage(LCD_CHILD_ID_pDoorFrontdoor,V_CUSTOM).set(isDoorFrontdoorClosed ? 1 : 0));
        break;
    }
    receivedRealMessage = false;    
  }
}

void setDisplaySleepStatus()
{
  // set display status and verify if current time between 22.00 and 07.00
  if ((!isGarageBikeClosed || !isGarageCarClosed || !isDoorHobbyClosed || !isDoorStorageClosed || !isDoorVerandaClosed || !isDoorFrontdoorClosed)
      && (hour() >= 22 || hour() <= 7))
  {
    sendCommand("sleep=0");  // exit sleep mode
    sendCommand("thsp=0"); // disable auto sleep
    isThspSetTo60 = false;
  }
  else
  {
    if (!isThspSetTo60)
    {
      sendCommand("thsp=60"); // auto sleep after 1 minute no touch
      isThspSetTo60 = true;
    } 
  }
}

void sendHeartbeatToController()
{
  // Send heartbeat 
  if ((millis() - lastHeartbeatTime) > HEARTBEAT_TIME) 
  {
    lastHeartbeatTime = millis();  
    if (sendHeartbeat())
    {
      statusRadioNetwork.Set_background_image_pic(NEXTION_PICTURE_ID_NETWORK);
    }
    else
    {
      statusRadioNetwork.Set_background_image_pic(NEXTION_PICTURE_ID_NO_NETWORK);      
    }  
  }
}

void RequestNodesStatusFromController()
{
  if ((millis() - lastRequestTime) > REQUEST_TIME) 
  {
    lastRequestTime = millis();  

    // Fetch last known statusses from controller
    request(LCD_CHILD_ID_pGarBike, V_CUSTOM);
    wait(500); // provide time for the node to receive the answer   
    request(LCD_CHILD_ID_pGarCar, V_CUSTOM);
    wait(500);    
    request(LCD_CHILD_ID_pDoorHobby, V_CUSTOM);
    wait(500);    
    request(LCD_CHILD_ID_pDoorStorage, V_CUSTOM);
    wait(500);    
    request(LCD_CHILD_ID_pDoorVeranda, V_CUSTOM);
    wait(500);    
    request(LCD_CHILD_ID_pDoorFrontdoor, V_CUSTOM);    
    wait(500);    
  }
}

// This is called when a new time value was received
void receiveTime(uint32_t time) {
  #ifdef MY_DEBUG
  Serial.println("Entering receive time method of MySensors...");
  #endif  

  // we are receiving radio messages
  statusRadioNetwork.Set_background_image_pic(NEXTION_PICTURE_ID_NETWORK);  
  
  // Ok, set incoming time 
  setTime(time);
  timeReceived = true;
  
  #ifdef MY_DEBUG
  Serial.print("The date is: ");Serial.print(day());Serial.print("-");Serial.print(month());Serial.print("-");Serial.println(year());
  Serial.print("The time is: ");Serial.print(hour());Serial.print(":");Serial.print(minute());Serial.print(":");Serial.println(second());
  #endif  
    
  String rtc = "rtc0=";
  String rtcx = rtc + year();
  sendCommand(rtcx.c_str());
  rtc = "rtc1=";
  rtcx = rtc + month();
  sendCommand(rtcx.c_str());
  rtc = "rtc2=";
  rtcx = rtc + day();
  sendCommand(rtcx.c_str());
  rtc = "rtc3=";
  rtcx = rtc + hour();  
  sendCommand(rtcx.c_str());
  rtc = "rtc4=";
  rtcx = rtc + minute();  
  sendCommand(rtcx.c_str());
  rtc = "rtc5=";
  rtcx = rtc + second();
  sendCommand(rtcx.c_str());    
}
