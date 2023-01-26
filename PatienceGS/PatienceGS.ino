// Patience Server code to transceive messages between the server, rocket, and launchpad.

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


int command = 0;

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
  float rpiRX = 9;
  float lpROLL;
  float lpPITCH;
  float lpYAW;
} gpsData;

void setup()
{

  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial & debug == true) {
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
  rf95.setTxPower(20, false);
}

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
  if (manager.available())
  {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      //delay(100);
      memcpy((uint8_t*)&gpsData, buf, sizeof(gpsData));
      delay(100);
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
    }
  }
  if (Serial.available() > 0) {
    command = Serial.read() - '0'; 
  }
  else if (Serial1.available() > 0) {
    command = Serial1.readString().toInt();
  }
  else {
    command = 0;
  }
  switch (command) {
    case 1: {
        gpsData.commandTX = command;
        if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), LAUNCHPAD_ADDRESS) || debug == true) {
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {

            Serial.println("\nCommunications Check Complete.");

          } else {
            Serial.println("\nRocket Communications not working...");
          }
        }
        else {
            Serial.println("\nLaunchpad Communications not working...");
        
        }
        break;
      }
    case 2: {
        gpsData.commandTX = command; 
        if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), LAUNCHPAD_ADDRESS)) {
            Serial.println("\n5....");
            delay(1000);
            Serial.println("\n4....");
            delay(1000);
            Serial.println("\n3....");
            delay(1000);
            Serial.println("\n2....");
            delay(1000);
            Serial.println("\n1....");
            delay(1000);
            Serial.println("\nIgnition.");
          }
        }
        break;
      }
    case 3: {
        gpsData.commandTX = command; 
        if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
          Serial.println("\nDeployment.");
        }
        break;
      }
    case 9: {
        gpsData.commandTX = command;
        if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
          Serial.println("\nReset.");
        }
        break;
      }
    default: {
        command = 0;
        break;
      }
  }

}
