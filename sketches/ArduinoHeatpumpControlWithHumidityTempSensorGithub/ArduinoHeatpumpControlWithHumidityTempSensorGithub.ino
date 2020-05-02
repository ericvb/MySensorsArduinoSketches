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
 * Version 1.0 - Toni A - https://github.com/ToniA/arduino-heatpumpir
 * Version 2.1 - Author unknown - example script from the Home Assistant website : https://www.home-assistant.io/integrations/climate.mysensors
 * Version 2.2 - Eric Van Bocxlaer - based on the example script from the Home Assistant website : https://www.home-assistant.io/integrations/climate.mysensors
 *                                 - https://community.home-assistant.io/t/mysensors-hvac-not-showing-up/22540
 * Version 2.3 - Eric Van Bocxlaer - adding the Air Humidity Sensor - DHT - sketch based on the work of cnerone and others: https://raw.githubusercontent.com/cnerone/MySensorsArduinoExamples/master/examples/DhtTemperatureAndHumiditySensor/DhtTemperatureAndHumiditySensor.ino
 *                                 - used smartSleep to replace wait() and sleep()  
 *                                 - correction code working with the offsets
 * DESCRIPTION
 * Heatpump controller + humidity and temperature sensor DHT
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG
// Enable specific RFM69 debug prints to serial monitor
//#define MY_DEBUG_VERBOSE_RFM69

#define MY_NODE_ID 3  // set the node ID manually because a MQTT gateway will not assign automatically a node Id - this must be set before the mysensors.h call

// Enable and select radio type attached. Replace the defines if you use other radio type hardware.
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_868MHZ // Set your frequency here
#define MY_IS_RFM69HW // Omit if your RFM is not "H"
#define MY_RFM69_NEW_DRIVER  // soft spi for rfm69 radio works only with new driver

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
#include <Adafruit_Sensor.h> // Official "Adafruit Unified Sensor" by Adafruit (tested on version 1.1.1)  http://librarymanager/all#adafruit
#include <DHT_U.h> // Official *DHT Sensor library* by Adafruit (tested on version 1.3.8)  http://librarymanager/all#dht

#define SENSOR_NAME "Heatpump Sensor and Humidity-Temperature sensor"
#define SENSOR_VERSION "2.3"

// ********************************************************DTH-11 sensor defines and variables*****************************************************************************
// Uncomment the type of sensor in use:
#define DHTTYPE           DHT11     // DHT 11 
//#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

// Sleep time between sensor updates (in milliseconds) to add to sensor delay (read from sensor data; typically: 1s)
static const uint64_t UPDATE_INTERVAL = 60000; 

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

// Set this to the pin you connected the DHT's data and power pins to; connect wires in coherent pins
#define DHTDATAPIN        4         
#define DHTPOWERPIN       8   // we will use a digital IO pin as power source of the DHT sensor

#define CHILD_ID_HUM 1
#define CHILD_ID_TEMP 2
#define CHILD_ID_HEATINDEX 3

// Set this offset if the sensors have permanent small offsets to the real temperatures/humidity.
// In Celsius degrees or moisture percent
#define SENSOR_HUM_OFFSET 0         // used for humidity data
#define SENSOR_TEMP_OFFSET 1.5      // used for temperature data and heat index computation
#define SENSOR_HEATINDEX_OFFSET 0   // used for heat index data

DHT_Unified dhtu(DHTDATAPIN, DHTTYPE);
// See guide for details on Adafruit sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

uint32_t delayMS;
float lastTemp;
float lastHum;
uint8_t nNoUpdates = FORCE_UPDATE_N_READS; // send data on start-up 
bool metric = true;
float temperature;
float humidity;
float heatindex;
// ********************************************************DTH-11 sensor defines*******************************************************************************************

#define CHILD_ID_HVAC 0 // Each radio node can report data for up to 254 different child sensors. You are free to choose the child id yourself. 
                        // You should avoid using child-id 255 because it is used for things like sending in battery level and other (protocol internal) node specific information.
                        
#define IR_PIN  3  // Arduino pin tied to the IR led using Arduino PWM


// Uncomment your heatpump model
//#include <FujitsuHeatpumpIR.h>
//#include <PanasonicCKPHeatpumpIR.h>
//#include <PanasonicHeatpumpIR.h>
//#include <CarrierHeatpumpIR.h>
//#include <MideaHeatpumpIR.h>
//#include <MitsubishiHeatpumpIR.h>
#include <SamsungHeatpumpIR.h>
//#include <SharpHeatpumpIR.h>
//#include <DaikinHeatpumpIR.h>

//Some global variables to hold the states
int POWER_STATE;
int TEMP_STATE;
int FAN_STATE;
int MODE_STATE;
int VDIR_STATE;
int HDIR_STATE;

IRSenderPWM irSender(IR_PIN);

//Change to your Heatpump
HeatpumpIR *heatpumpIR = new SamsungAQVHeatpumpIR();
/*
new PanasonicDKEHeatpumpIR()
new PanasonicJKEHeatpumpIR()
new PanasonicNKEHeatpumpIR()
new CarrierHeatpumpIR()
new MideaHeatpumpIR()
new FujitsuHeatpumpIR()
new MitsubishiFDHeatpumpIR()
new MitsubishiFEHeatpumpIR()
new SamsungAQVHeatpumpIR()
new SamsungFJMHeatpumpIR()
// new SamsungHeatpumpIR() is a protected generic method, cannot be created directly
new SharpHeatpumpIR()
new DaikinHeatpumpIR()
*/

MyMessage msgHVACSetPointC(CHILD_ID_HVAC, V_HVAC_SETPOINT_COOL);
MyMessage msgHVACSpeed(CHILD_ID_HVAC, V_HVAC_SPEED);
MyMessage msgHVACFlowState(CHILD_ID_HVAC, V_HVAC_FLOW_STATE);

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHeatIndex(CHILD_ID_HEATINDEX, V_TEMP);

bool initialValueSent = false;

float computeHeatIndex(float temperature, float percentHumidity) {
  // Based on Adafruit DHT official library (https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp)
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

  float hi;

  temperature = temperature + SENSOR_TEMP_OFFSET; //include TEMP_OFFSET in HeatIndex computation too
  temperature = 1.8*temperature+32; //convertion to *F

  percentHumidity = percentHumidity + SENSOR_HUM_OFFSET; // include HUM_OFFSET
  
  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  hi = (hi-32)/1.8;
  return hi; //return Heat Index, in *C
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SENSOR_NAME, SENSOR_VERSION);

  // Register all sensors to gw (they will be created as child devices) by their ID and S_TYPE
  present(CHILD_ID_HVAC, S_HVAC, "Thermostat");
  present(CHILD_ID_HUM, S_HUM, "Humidity");
  present(CHILD_ID_TEMP, S_TEMP, "Temperature");
  present(CHILD_ID_HEATINDEX, S_TEMP, "Heat Index");
  metric = getControllerConfig().isMetric;
}

void setup() {
  pinMode(DHTPOWERPIN, OUTPUT);
  digitalWrite(DHTPOWERPIN, HIGH);
  // Initialize device.
  dhtu.begin();
  
  Serial.println("DHTxx Unified Sensor");
  // Print temperature sensor details.
  sensor_t sensor;
  dhtu.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.print  ("Min Delay:   "); Serial.print(sensor.min_delay/1000); Serial.println(" ms");  
  Serial.println("------------------------------------");
  
  // Print humidity sensor details.
  dhtu.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.print  ("Min Delay:   "); Serial.print(sensor.min_delay/1000); Serial.println(" ms");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000; 
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!initialValueSent) {
    send(msgHVACSetPointC.set(20));
    send(msgHVACSpeed.set("Auto"));
    send(msgHVACFlowState.set("Off"));

    initialValueSent = true;
  }

  //********************************DHT-11 handling code ********************************
  digitalWrite(DHTPOWERPIN, HIGH);   
  delay(delayMS);
  sensors_event_t event;  
  // Get temperature event and use its value.
  dhtu.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    temperature = event.temperature;
    #ifdef MY_DEBUG
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
    #endif
  }

  // Get humidity event and use its value.
  dhtu.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    humidity = event.relative_humidity;
    #ifdef MY_DEBUG
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    #endif
  }

  if (fabs(humidity - lastHum)>=0.05 || fabs(temperature - lastTemp)>=0.05 || nNoUpdates >= FORCE_UPDATE_N_READS) {
    lastTemp = temperature;
    lastHum = humidity;
    heatindex = computeHeatIndex(temperature,humidity); //computes Heat Index, in *C
    nNoUpdates = 0; // Reset no updates counter
    #ifdef MY_DEBUG
    Serial.print("Heat Index: ");
    Serial.print(heatindex);
    Serial.println(" *C");    
    #endif    
    
    if (!metric) {
      temperature = 1.8*temperature+32; //convertion to *F
      heatindex = 1.8*heatindex+32; //convertion to *F
    }
    
    #ifdef MY_DEBUG
    wait(100);
    Serial.print("Sending temperature: ");
    Serial.print(temperature);
    #endif    
    send(msgTemp.set(temperature + SENSOR_TEMP_OFFSET, 2));

    #ifdef MY_DEBUG
    wait(100);
    Serial.print("Sending humidity: ");
    Serial.print(humidity);
    #endif    
    send(msgHum.set(humidity + SENSOR_HUM_OFFSET, 2));

    #ifdef MY_DEBUG
    wait(100);
    Serial.print("Sending HeatIndex: ");
    Serial.print(heatindex);
    #endif    
    send(msgHeatIndex.set(heatindex + SENSOR_HEATINDEX_OFFSET, 2));
  }

  nNoUpdates++;

  // Sleep for a while to save energy
  digitalWrite(DHTPOWERPIN, LOW); 
  smartSleep(UPDATE_INTERVAL); //sends heartbeat and process incoming messages before going to sleep    
}

void receive(const MyMessage &message) {
  if (message.isAck()) {
     Serial.println("This is an ack from gateway");
     return;
  }

  Serial.print("Incoming message for: ");
  Serial.print(message.sensor);

  String recvData = message.data;
  recvData.trim();

  Serial.print(", New status: ");
  Serial.println(recvData);
  switch (message.type) {
    case V_HVAC_SPEED:
      Serial.println("V_HVAC_SPEED");

      if(recvData.equalsIgnoreCase("auto")) FAN_STATE = 0;
      else if(recvData.equalsIgnoreCase("min")) FAN_STATE = 1;
      else if(recvData.equalsIgnoreCase("normal")) FAN_STATE = 2;
      else if(recvData.equalsIgnoreCase("max")) FAN_STATE = 3;
    break;

    case V_HVAC_SETPOINT_COOL:
      Serial.println("V_HVAC_SETPOINT_COOL");
      TEMP_STATE = message.getFloat();
      Serial.println(TEMP_STATE);
    break;

    case V_HVAC_FLOW_STATE:
      Serial.println("V_HVAC_FLOW_STATE");
      if (recvData.equalsIgnoreCase("coolon")) {
        POWER_STATE = 1;
        MODE_STATE = MODE_COOL;
      }
      else if (recvData.equalsIgnoreCase("heaton")) {
        POWER_STATE = 1;
        MODE_STATE = MODE_HEAT;
      }
      else if (recvData.equalsIgnoreCase("autochangeover")) {
        POWER_STATE = 1;
        MODE_STATE = MODE_AUTO;
      }
      else if (recvData.equalsIgnoreCase("off")){
        POWER_STATE = 0;
      }
      break;
  }
  sendHeatpumpCommand();
  sendNewStateToGateway();
}

void sendNewStateToGateway() {
  send(msgHVACSetPointC.set(TEMP_STATE));
  send(msgHVACSpeed.set(FAN_STATE));
  send(msgHVACFlowState.set(MODE_STATE));
}

void sendHeatpumpCommand() {
  Serial.println("Power = " + (String)POWER_STATE);
  Serial.println("Mode = " + (String)MODE_STATE);
  Serial.println("Fan = " + (String)FAN_STATE);
  Serial.println("Temp = " + (String)TEMP_STATE);

  heatpumpIR->send(irSender, POWER_STATE, MODE_STATE, FAN_STATE, TEMP_STATE, VDIR_AUTO, HDIR_AUTO);
}
