#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "Adafruit_FRAM_SPI.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
//#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#define RFM95_INT 7
#define RFM95_CS 10 //8
#define RFM95_RST 11 //9
#define GPSSerial Serial2
#define GPSECHO false
#define LED 25
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define BROADCAST_ADDRESS 255
#define RF95_FREQ 915.0
#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
#define sendcycle 1330
#define debug false
//#define AHRS_DEBUG_OUTPUT
Adafruit_GPS GPS(&GPSSerial);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, CLIENT_ADDRESS);
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
Adafruit_NXPSensorFusion filter; // slowest
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

uint32_t timer = millis();
int deployCheck = 0;
int ignitCheck = 0;
int sysCheck = 0;
uint32_t timestamp;
uint8_t FRAM_CS = 13;
uint8_t FRAM_SCK = 14;
uint8_t FRAM_MISO = 12;
uint8_t FRAM_MOSI = 15;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint16_t          addr = 1;
// software SPI, any pins!
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

int LED_Switch = 0;
int counter = 0;
int Cycle = 0;
struct dataStruct {
  float latitudeGPS;// = 1111.111111;
  float longitudeGPS;// = 1111.111111;
  char latGPS;// = {'1'};
  char lonGPS;// = {'1'};
  float altitudeVEC;//=111.1;
  int commandRX = 9;
  float ROLL;
  float PITCH;
  float YAW;
} gpsData;


//#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
//#else
//Adafruit_Sensor_Calibration_SDFat cal;
//#endif

bool init_sensors(void) {
  if (!fxos.begin() || !fxas.begin()) {
    return false;
  }
  accelerometer = fxos.getAccelerometerSensor();
  gyroscope = &fxas;
  magnetometer = fxos.getMagnetometerSensor();

  return true;
}

void setup_sensors(void) {}

void setup() {
  GPSSerial.setTX(8); //(4);
  GPSSerial.setRX(9); //(5);
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  while (!Serial & debug == true) {
    yield();
  }
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
  if (!rf95.init()) {
    Serial.println("Comms initilization failed.");
  }
//  if (!rf95.setFrequency(RF95_FREQ)) {
//    Serial.println("setFrequency failed");
//    //    //    while (1);
//  }
//  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(20, false);
  // gpsData.commandTX = 0;
  if (!manager.init()) {
    Serial.println("init failed");
  }
  if (fram.begin()) {
    Serial.println("Found SPI FRAM");
  }
  else {
    Serial.println("No SPI FRAM found ... check your connections\r\n");
  //  while (1);
  }
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  }
  if (!cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  else  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz

}

void loop() {
  while (1) {

    aqAHRS();
    //delay(10);
    if (((abs(gpsData.ROLL) >= 80) || (abs(gpsData.PITCH) >= 80)) & deployCheck == 0 & sysCheck == 1 ) {
      gpsData.commandRX = 3;
      deployCheck = 1;
    }
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
    uint8_t len = sizeof(buf);

    //     if (rf95.available()) {
    if (manager.available()) {
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAck(buf, &len, &from)) {
        //        if (rf95.recv(buf, &len)) {
        memcpy(&gpsData, buf, sizeof(gpsData));
        // delay(10);
        Serial.println("Receiving Command");
        //rf95.waitPacketSent();
      }
    }
    //delay(500);
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
