// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX

#include <SPI.h>
#include <RH_RF95.h>

//#include <ICSP.h>
#define RFM95_CS 12
#define RFM95_RST 13
#define RFM95_INT 11

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(115200);
  Serial1.begin(115200);
  //  Serial2.begin(115200);
  //  Serial3.begin(115200);



  delay(100);

  Serial.println("Arduino LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}
int Cycle = 0;
uint8_t mesg[] = "                       ";
void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    int MsgCycle = Cycle % 2;
    if (strcmp((char*)buf, "Reset") == 0)
    {
      Cycle += 1;
    }
//    Cycle += 1;
    switch (MsgCycle) {
      case 0: {
          if (rf95.recv(buf, &len))
          {
            digitalWrite(LED, HIGH);
            //RH_RF95::printBuffer("Received: ", buf, len);
            Serial.print("Got: ");
            Serial.println((char*) buf);
            Serial1.println((char*) buf);
            //      if ( strstr((char*)buf, "Communications online"))
            //
            //
            //                  //      Serial.print("RSSI: ");
            //                  //      Serial.println(rf95.lastRssi(), DEC);
            //
            //                  // Send a reply
            //                  uint8_t data[] = "And hello back to you";
            //                  rf95.send(data, sizeof(data));
            //                  rf95.waitPacketSent();
            //                  //      Serial.println("Sent a reply");
            //                  digitalWrite(LED, LOW);
            //                }
            //                  else
            //                  {
            //                  Serial.println("Receive failed");
            //                }
          }
          break;
        }
      case 1:
        if (rf95.recv(buf, &len))
        {
          digitalWrite(LED, HIGH);
          //RH_RF95::printBuffer("Received: ", buf, len);
          Serial.print("Got1: ");
          Serial.println((char*) buf);
          Serial1.println((char*) buf);
        }
        break;
    }
  }

  if (Serial.available() > 0) {
    int command = Serial.read() - '0';
    //coder = ";
    //uint8_t coderlen = sizeof(coder);
    //Serial1.write(coder); //, &coderlen);
    //    Serial1.println((char*) buf);
    switch (command)
    {
      case 1: {
          // Serial.print("Ready for launch.");
          uint8_t mesg[] = "1";
          rf95.send(mesg, sizeof(mesg));
          break;
        }
      case 2:
        {
          // Serial.print("Blast off!");
          uint8_t mesg[] = "Blast Off!";
          rf95.send(mesg, sizeof(mesg));
          break;
        }
      case 3:
        {
          // Serial.print("Emergency Deployment!");
          uint8_t mesg[] = "Deployment";
          rf95.send(mesg, sizeof(mesg));
          break;
        }
      case 4:
        {
          uint8_t mesg[] = "Reset";
          rf95.send(mesg, sizeof(mesg));
        }
      default:
        break;
    }
  }
}
