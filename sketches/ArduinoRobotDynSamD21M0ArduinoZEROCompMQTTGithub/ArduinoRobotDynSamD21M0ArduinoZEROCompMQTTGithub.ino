/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
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
 * Version 1.1 - Eric Van Bocxlaer (added support for RFM69HW and EthernetShieldW5500 through the SPI bus on a RobotDyn SAMD21-M0 or Arduino mZero compatible board)
 *               Install in the Arduino IDE, via the Board Manager, the package "Arduino SAMD Boards (32-bit ARM Cortex-M0+)". Choose the board "Arduino M0".
 *               Be aware that the RobotDyn board is not completely pinlayout compatible with mZero!! 
 *               See https://forum.mysensors.org/topic/10565/solved-arduino-mzero-clone-robotdyn-samd21-m0-board-with-rfm69hw-sending-but-not-receiving-parent-response
 *
 * DESCRIPTION
 * The W5500 MQTT gateway sends radio network (or locally attached sensors) data to your MQTT broker.
 * The node also listens to MY_MQTT_TOPIC_PREFIX and sends out those messages to the radio network
 *
 * LED purposes:
 * - To use the feature, uncomment WITH_LEDS_BLINKING in MyConfig.h
 * - RX (green) - blink fast on radio message received. In inclusion mode will blink fast only on presentation received
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or receive crc error
 *
 * Connection with RFM69HW and EthernetShieldW5500 through the SPI bus (Serial Peripheral Interface)
 * On this bus we can have only one master (= the Arduino Zero or compatible board) and different slaves (= the W5500 and the RFM69HW).
 * The master can only talk to one slave at a time!
 * 
 * RFM69HW    mZERO        Ethernet W5500
 * VCC 3.3V   VCC 3.3V    VCC 3.3V
 * DIO0       D8
 * NSS        D7
 * SCK        Header/SCK  SCK
 * MISO       Header/MISO MISO
 * MOSI       Header/MOSI MOSI
 *            D10         SCS (W5500)(Default in ethernet library: do not wire different if you are using a 'no shield version'!)
 *            
 * 
 * NSS/SS/SCS : Slave Select for SPI bus (the master selects with which slave he want to talk)
 * SCK : serial clock signal delivered by the master
 * MISO : Master Input Slave Output (arduino receives data from the slave)
 * MOSI : Master Output Slave Input (arduino sends data to the slave)
 * 
 * See also https://www.arduino.cc/en/Reference/SPI for the different connections depending on the type of Arduino boards
 */


// Enable debug prints to serial monitor
//#define MY_DEBUG
// Enable specific RFM69 debug prints to serial monitor
//#define MY_DEBUG_VERBOSE_RFM69

// Enables and select radio type. Replace the defines if you use other radio type hardware.
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_868MHZ // Set your frequency here
#define MY_IS_RFM69HW // Omit if your RFM is not "H"
#define MY_RFM69_NEW_DRIVER

#define MY_GATEWAY_MQTT_CLIENT

// Set this node's subscribe and publish topic prefix
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mysensorsgateway-out"
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mysensorsgateway-in"

// Set MQTT client id
#define MY_MQTT_CLIENT_ID "mysensors-1"

// Enable gateway ethernet module type : not needed here because MY_GATEWAY_MQTT_CLIENT defines already the use of W5100
//#define MY_GATEWAY_W5100

// W5100 Ethernet module SPI enable (this setting is optional if using a shield/module that manages SPI_EN signal)
//#define MY_W5100_SPI_EN 4

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// The W5100 ethernet module seems to have a hard time co-operate with radio on the same spi bus.
// Situation on 2019-05-04: with the new RFM69HW driver this is not true anymore for a radio RFM69!
//#if !defined(MY_W5100_SPI_EN) && !defined(ARDUINO_ARCH_SAMD)
//#define MY_SOFTSPI
//#define MY_SOFT_SPI_SCK_PIN 14
//#define MY_SOFT_SPI_MISO_PIN 16
//#define MY_SOFT_SPI_MOSI_PIN 15
//#endif

// When a W5500 shield is connected, we have to move the SS (slave select) pin for the RFM69 radio because the shield
// is using the default D10
#define MY_RFM69_CS_PIN 7
#define MY_RFM69_IRQ_PIN 8

// Enable these if your MQTT broker requires username/password
#define MY_MQTT_USER "yourMQTTusername"
#define MY_MQTT_PASSWORD "yourMQTTsecretpassword"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,178,87

// If using static ip you can define Gateway and Subnet address as well
//#define MY_IP_GATEWAY_ADDRESS 192,168,178,1
//#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// MQTT broker ip address or url. Set here the IP or URL of your MQTT broker. Define only one of them!
//#define MY_CONTROLLER_URL_ADDRESS "m20.cloudmqtt.com"
#define MY_CONTROLLER_IP_ADDRESS 192, 168, 01, 05

// The MQTT broker port to to open
// Is a secure SSL connection between the Arduino MQTT gateway and the MQTT broker server possible?
// https://forum.mysensors.org/topic/10568/secure-ssl-connection-between-the-arduino-mqtt-gateway-and-the-mqtt-broker-server-status/2
// https://github.com/mysensors/MySensors/issues/1283
// https://github.com/mysensors/MySensors/pull/1387
// So yes, it will be available with MySensors 2.4.0...
// Remark : with simple Arduino UNO's we can't use encrypted communication anyway!
// So in the Mosquitto broker a specific port 1884 was added
#define MY_PORT 1884

/*
// Inclusion mode is not supported by the Home Assistant controller
// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3
*/

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300
// Flash leds on rx/tx/err
#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  5  // Transmit led pin

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

#include <Ethernet.h> // sketch tested with version 2.0.0, see http://librarymanager#Ethernet

// https://forum.mysensors.org/topic/10548/sensbender-gateway-gives-a-lot-of-hu
// temporary fix, can be deleted when Arduino SAMD board definitions will be corrected.
// #undef PRId8
// #undef PRIi8
// #undef PRIo8
// #undef PRIu8
// #undef PRIx8
// #undef PRIX8
// #undef PRIdLEAST8
// #undef PRIiLEAST8
// #undef PRIoLEAST8
// #undef PRIuLEAST8
// #undef PRIxLEAST8
// #undef PRIXLEAST8
// #undef PRIdFAST8
// #undef PRIiFAST8
// #undef PRIoFAST8
// #undef PRIuFAST8
// #undef PRIxFAST8
// #undef PRIXFAST8
// #define PRId8    "d"
// #define PRIi8   "i"
// #define PRIo8   "o"
// #define PRIu8   "u"
// #define PRIx8   "x"
// #define PRIX8   "X"
// #define PRIdLEAST8  "d"
// #define PRIiLEAST8  "i"
// #define PRIoLEAST8  "o"
// #define PRIuLEAST8  "u"
// #define PRIxLEAST8  "x"
// #define PRIXLEAST8  "X"
// #define PRIdFAST8 "d"
// #define PRIiFAST8 "i"
// #define PRIoFAST8 "o"
// #define PRIuFAST8 "u"
// #define PRIxFAST8 "x"
// #define PRIXFAST8 "X"
// not needed anymore when working with Arduino IDE >= 1.18.12

#include <MySensors.h> // sketch tested with version 2.3.2, see http://librarymanager#MySensors

void setup()
{
    // Setup locally attached sensors
}

void presentation()
{
    // Present locally attached sensors here
}

void loop()
{
    // Send locally attached sensors data here
}
