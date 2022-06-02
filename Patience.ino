#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_INT 7
#define RFM95_CS 8
#define RFM95_RST 9
#define GPSSerial Serial2
#define GPSECHO false
#define LED 25
Adafruit_GPS GPS(&GPSSerial);
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint32_t timer = millis();
uint32_t timer2 = millis();
uint32_t timer3 = millis();
uint32_t timer4 = millis();
uint32_t timer5 = millis();
uint32_t timer6 = millis();
int LED_Switch = 0;
int counter = 0;
int Cycle = 0;
struct dataStruct {
  float latitudeGPS;
  float longitudeGPS;
  char latGPS;
  char lonGPS;
  float altitudeVEC;
  int commandRX = 9;
} gpsData;

void setup() {
  GPSSerial.setTX(4);
  GPSSerial.setRX(5);
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  //  while (!Serial); {
  //    yield();
  //  }
  Serial.println("Adafruit GPS library basic parsing test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(100);
  GPSSerial.println(PMTK_Q_RELEASE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  Serial.println("Arduino LoRa RX Test!");
  if (!rf95.init())
    Serial.println("Comms initilization failed.");
  rf95.setTxPower(23, false);
  // gpsData.commandTX = 0;
}

void loop() {
  while (1) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
    uint8_t len = sizeof(buf);

    if (rf95.available()) {
      if (rf95.recv(buf, &len)) {
        memcpy(&gpsData, buf, sizeof(gpsData));
        delay(10);
        Serial.println("Receiving");
        //rf95.waitPacketSent();
      }
    }
    //gpsData.commandTX = 0;
    sendGPS();
    Commands();
    if (LED_Switch == 1) {
      digitalWrite(LED, HIGH);
    }
    else {
      digitalWrite(LED, LOW);
    }
  }
}
