// rf95_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS 12
#define RFM95_RST 13
#define RFM95_INT 11
#define RF95_FREQ 902.3
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define LAUNCHPAD_ADDRESS 3
#define BROADCAST_ADDRESS 255
#define debug false
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, SERVER_ADDRESS);

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB
struct dataStruct {
  float latitudeGPS;
  float longitudeGPS;
  char latGPS;
  char lonGPS;
  float altitudeGPS;
  int commandTX = 9;
  float ROLL;
  float PITCH;
  float YAW;
} gpsData;

void setup()
{
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  //  pinMode(4, OUTPUT);
  //  digitalWrite(4, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial) {
    yield() ; // Wait for serial port to be available
  }
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  if (!manager.init()) {
    Serial.println("init failed");
  } else {
    Serial.println("\nReliable datagram init OK!");
  }
  delay(10);
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
        while (1);
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(20, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  driver.setTxPower(14, true);
  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
  //  driver.setCADTimeout(10000);
}

// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
  if (manager.available())
  {
    //Serial.println("Manager is available");
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      //delay(100);
      memcpy((uint8_t*)&gpsData, buf, sizeof(gpsData));
      delay(100);
      //      Serial.print("got request from : 0x");
      //      Serial.print(from, HEX);
      //      Serial.print(": ");
      //      Serial.println(gpsData.longitudeGPS);
      if (debug == true) {
        Serial.print(gpsData.latitudeGPS, 4); Serial.print('\n');
        Serial.print(gpsData.longitudeGPS, 4); Serial.print('\n');
        Serial.print(gpsData.latGPS); Serial.print('\n');
        Serial.print(gpsData.lonGPS); Serial.print('\n');
        Serial.print(gpsData.altitudeGPS, 1); Serial.print('\n');
        Serial.print(gpsData.commandTX, DEC); Serial.print('\n');
        Serial.print(gpsData.ROLL, 2); Serial.print('\n');
        Serial.print(gpsData.PITCH, 2); Serial.print('\n');
        Serial.print(gpsData.YAW, 2); Serial.print('\n');
      }
      Serial.print("...");
      Serial1.print(gpsData.latitudeGPS, 4); Serial1.print('\n');
      Serial1.print(gpsData.longitudeGPS, 4); Serial1.print('\n');
      Serial1.print(gpsData.latGPS); Serial1.print('\n');
      Serial1.print(gpsData.lonGPS); Serial1.print('\n');
      Serial1.print(gpsData.altitudeGPS, 1); Serial1.print('\n');
      Serial1.print(gpsData.commandTX, DEC); Serial1.print('\n');
      Serial1.print(gpsData.ROLL, 2); Serial1.print('\n');
      Serial1.print(gpsData.PITCH, 2); Serial1.print('\n');
      Serial1.print(gpsData.YAW, 2); Serial1.print('\n');
      // Send a reply back to the originator client
      //      if (!manager.sendtoWait(data, sizeof(data), from))
      //        Serial.println("sendtoWait failed");
    }
  }
  if (Serial.available() > 0) {
    int command = Serial.read() - '0'; //(BUFFER_SIZE);
    //Serial.print(command);
    switch (command) {
      case 1: {
          //        Cycle = 1;
          gpsData.commandTX = command; //Cycle;
          //Serial.println("W");
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), LAUNCHPAD_ADDRESS)) {
            if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {

            Serial.println("\nCommunications Check Complete.");

          } else {
            Serial.println("\nCommunications not working...");
          }
          }
          break;
        }
      case 2: {
          //          Cycle = 2;
          gpsData.commandTX = command; //int(Serial.read() /10);
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
            if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), LAUNCHPAD_ADDRESS)) {
              Serial.println("\nIgnition.");
            }
          }
          break;
        }
      case 3: {
          //            Cycle = 3;
          gpsData.commandTX = command; //int(Serial.read() /10);
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
            Serial.println("\nDeployment.");
          }
          break;
        }
      case 9: {
          //              Cycle = 9;
          gpsData.commandTX = command;
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
            Serial.println("\nReset.");
          }
          break;
        }
      default: {
          break;
        }
    }
  }

}
