// rf95_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
//#include <ICSP.h>
#define RFM95_CS 12
#define RFM95_RST 13
#define RFM95_INT 11
#define RF95_FREQ 902.3
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define BROADCAST_ADDRESS 255
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, SERVER_ADDRESS);
// Blinky on receipt
//#define LED 13
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

  //  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  //Serial1.begin(115200);
  while (!Serial) ; // Wait for serial port to be available
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  if (!rf95.init()) {
    Serial.println("init failed");
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  if (!manager.init()) {
    Serial.println("init failed");
  } else {
    Serial.println("\nReliable datagram init OK!");
  }
//
      if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
    //    while (1);
      }
  //    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(20, false);
  //  Wire.setClock(400000); // 400KHz
}

int Cycle = 0;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t from;
    uint8_t len = sizeof(buf);

void loop() {
 // while(1) {
  //uint8_t mesg[] = {};
  //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
  delay(10);
  //uint8_t len = sizeof(buf);
  if (manager.available()) {
    // if (rf95.available()){
    if (manager.recvfromAck(buf, &len, &from))
      //  if (rf95.recv(buf, &len))
    {
      //  digitalWrite(LED, HIGH);

      Serial.print("...");
     // memcpy(&gpsData, buf, sizeof(buf));
      //delay(100);
//      Serial1.print(gpsData.latitudeGPS, 6); Serial1.print('\n');
//      Serial1.print(gpsData.longitudeGPS, 6); Serial1.print('\n');
//      Serial1.print(gpsData.latGPS); Serial1.print('\n');
//      Serial1.print(gpsData.lonGPS); Serial1.print('\n');
//      Serial1.print(gpsData.altitudeGPS, 1); Serial1.print('\n');
//      Serial1.print(gpsData.commandTX, DEC); Serial1.print('\n');
//      Serial1.print(gpsData.ROLL, 2); Serial1.print('\n');
//      Serial1.print(gpsData.PITCH, 2); Serial1.print('\n');
//      Serial1.print(gpsData.YAW, 2); Serial1.print('\n');
      //        if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), from)) {
      //          Serial.println("...");
      //        }
    }
    else{
      Serial.println("");
    }
  }
  //delay(10);
  //    if (Serial.available() > 0) {
  //      int command = Serial.read() - '0'; //(BUFFER_SIZE);
  //      //Serial.print(command);
  //      switch (command) {
  //        case 1: {
  //            Cycle = 1;
  //            gpsData.commandTX = command; //Cycle;
  //            //Serial.println("W");
  //            if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
  //
  //              Serial.println("\nCommunications Check Complete.");
  //
  //            } else {
  //              Serial.println("\nCommunications not working...");
  //            }
  //            break;
  //          }
  //        case 2: {
  //            Cycle = 2;
  //            gpsData.commandTX = command; //int(Serial.read() /10);
  //            if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
  //              if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), BROADCAST_ADDRESS)) {
  //                Serial.println("\nIgnition.");
  //              }
  //            }
  //            break;
  //          }
  //        case 3: {
  //            Cycle = 3;
  //            gpsData.commandTX = command; //int(Serial.read() /10);
  //            if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
  //              Serial.println("\nDeployment.");
  //            }
  //            break;
  //          }
  //        case 9: {
  //            Cycle = 9;
  //            gpsData.commandTX = command;
  //            if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
  //              Serial.println("\nReset.");
  //            }
  //            break;
  //          }
  //        default: {
  //            break;
  //          }
  //      }
  //    }

  //    if (Serial1.available() > 0) {
  //      Serial.print(Serial1.read());
  // }
  //}

}
