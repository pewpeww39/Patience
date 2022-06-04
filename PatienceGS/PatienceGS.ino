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
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
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
} gpsData;

void setup()
{
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  //  pinMode(4, OUTPUT);
  //  digitalWrite(4, HIGH);

  //  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial) ; // Wait for serial port to be available
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  if (!rf95.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  if (!manager.init())
    Serial.println("init failed");
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  driver.setTxPower(14, true);
  //Serial.println("Latitude, Longitude, Altitude");
}


//float altitudeLast;
int Cycle = 0;
//const int BUFFER_SIZE = 1;
//char buf[BUFFER_SIZE];
//int command = {1};
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
void loop()
{

  // Should be a message for us now
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
  uint8_t mesg[] = {};
  uint8_t len = sizeof(buf);
  //int MsgCycle = 0; //Cycle % 2;
  //Cycle = Cycle + 1;
  //Serial.println(MsgCycle);
  //switch (MsgCycle) {
  //case 0: {
  if (manager.available())
    //if (rf95.available())
  {
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
      //if (rf95.recv(buf, &len))
    {
      //  digitalWrite(LED, HIGH);
      //      RH_RF95::printBuffer("request: ", buf, len);
      memcpy(&gpsData, buf, sizeof(gpsData));

      delay(10);
      //if (Serial.available())
      //{
      Serial1.print(gpsData.latitudeGPS, 6);
      Serial1.print(gpsData.longitudeGPS, 6);
      Serial1.print(gpsData.latGPS);
      Serial1.print(gpsData.lonGPS);
      Serial1.print(gpsData.altitudeGPS, 1);
      Serial1.print(gpsData.commandTX, DEC);
      //      rf95.send((uint8_t*)&gpsData, size3of(gpsData));
      //      rf95.waitPacketSent();
      // Send a reply back to the originator client
      if (!manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), from))
        Serial.println("sendtoWait failed");
    }

  }

  if (Serial.available() > 0) {
    int command = Serial.read() - '0'; //(BUFFER_SIZE);
    //Serial.print(command);
    switch (command) {
      case 1: {
          //Serial.println(command);//int(Serial.read() /10);
          //Serial.println(gpsData.commandTX);
          Cycle = 1;
          gpsData.commandTX = Cycle;
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
            uint8_t len = sizeof(buf);
            uint8_t from;
            delay(10);
            if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
              //rf95.waitPacketSent();
              Serial.println("Communications Check Complete.");
            }
          }
          //rf95.send((uint8_t*)&gpsData, sizeof(gpsData));
          //rf95.waitPacketSent();
          break;
        }
      case 2: {
          Cycle = 2;
          gpsData.commandTX = Cycle; //int(Serial.read() /10);
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
            uint8_t len = sizeof(buf);
            uint8_t from;
            delay(10);
            if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
              //rf95.waitPacketSent();
              Serial.println("Ignition.");
            }
          }//Serial.println(gpsData.commandTX);
          //rf95.send((uint8_t*)&gpsData, sizeof(gpsData));
          //rf95.waitPacketSent();
          break;
        }
      case 3: {
          Cycle = 3;
          gpsData.commandTX = Cycle; //int(Serial.read() /10);
          if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), CLIENT_ADDRESS)) {
            uint8_t len = sizeof(buf);
            uint8_t from;
            delay(10);
            if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
              //rf95.waitPacketSent();
              Serial.println("Deployment.");
            }
          }
          //Serial.println(gpsData.commandTX);
          //rf95.send((uint8_t*)&gpsData, sizeof(gpsData));
          //rf95.waitPacketSent();
          break;
        }
      default: {
          //Cycle = 0;
          //gpsData.commandTX = Cycle;
          //rf95.send((uint8_t*)&gpsData, sizeof(gpsData));
          //rf95.waitPacketSent();
          break;
        }

        //else if (strcmp((char*)command, "start") == 0)
        //{
        // Cycle = 2;
        //}
        //    else { Cycle = 9;}

        //Serial.print(command);
    }
  }
}
//      break;

//   case 1: {

//if (Serial.available() > 0) {
//int command = 1; //Serial.read() - '0';
//uint8_t command[RH_RF95_MAX_MESSAGE_LEN] = {Serial.read()};
//gpsData.commandTX = command;
//         rf95.send(mesg, sizeof(mesg));
//            rf95.waitPacketSent();
//Serial.println(gpsData.commandTX);
//Serial.println(command);

//                }
//  }
//
//        gpsData.commandTX = {1};
//
//        rf95.send((uint8_t *)&gpsData, sizeof(gpsData));
//        rf95.waitPacketSent();
//
//        Serial.print(gpsData.commandTX);
//
//        //          if (Cycle % 2 == 0){
//        //          }
//        break;
//      }
// }
//}

//coder = ";
//uint8_t coderlen = sizeof(coder);
//Serial1.write(coder); //, &coderlen);
//              Serial1.println((char*) buf);
//          uint8_t retmesg[4] = {};
//          uint8_t rtmlen = sizeof(retmesg);
//          switch (command)
//          {
//            case 1: {
//                // Serial.print("Ready for launch.");
//                uint8_t mesg[1] = {1};
//                gpsData.commandTX = mesg;
//                //            rf95.send(mesg, sizeof(mesg));
//                //            rf95.waitPacketSent();
//                Serial.println(gpsData.commandTX);
//                //if (!rf95.recv(retmesg, &rtmlen)) {
//                //yield();
//                //}
//                //else {
//                  //Serial.println("got: ");
//                  //Serial.println((char*)retmesg);
//                  uint8_t mesg[] = {};
//                }
//
//                break;
//              }
//            case 2:
//              {
//                // Serial.print("Blast off!");
//                uint8_t mesg[] = {"2"};
//                rf95.send(mesg, sizeof(mesg));
//                rf95.waitPacketSent();
//                if (rf95.recv(retmesg, &rtmlen)) {
//                  Serial.println((char*)retmesg);
//                }
//                break;
//              }
//            case 3:
//              {
//                // Serial.print("Emergency Deployment!");
//                uint8_t mesg[2] = {"3"};
//                rf95.send(mesg, sizeof(mesg));
//                rf95.waitPacketSent();
//                if (rf95.recv(retmesg, &rtmlen)) {
//                  Serial.println((char*)retmesg);
//                }
//                break;
//              }
//            case 4:
//              {
//                uint8_t mesg[2] = {"0"};
//                rf95.send(mesg, sizeof(mesg));
//                rf95.waitPacketSent();
//                if (rf95.recv(retmesg, &rtmlen)) {
//                  Serial.println((char*)retmesg);
