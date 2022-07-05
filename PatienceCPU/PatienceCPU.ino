// rf95_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define LAUNCHPAD_ADDRESS 3
#define BROADCAST_ADDRESS 255
#define RF95_FREQ 902.3
#define RFM95_INT 7
#define RFM95_CS 10 //8
#define RFM95_RST 11 //9
#define GPSSerial Serial2
#define GPSECHO false
#define LED 25
#define RF95_FREQ 902.3
#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
#define debug false
#define GP17 17

Adafruit_GPS GPS(&GPSSerial);
RH_RF95 driver(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
Adafruit_NXPSensorFusion filter; // slowest
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

uint32_t timer = millis();
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint16_t          addr = 1;
uint32_t timestamp;
int counter = 0;
int LED_Switch = 0;
int deployCheck = 0;
int ignitCheck = 0;
int sysCheck = 0;
int sendcycle = 1500;
struct dataStruct {
  float latitudeGPS;// = 1111.111111;
  float longitudeGPS;// = 1111.111111;
  char latGPS;// = {'1'};
  char lonGPS;// = {'1'};
  float altitudeGPS;//=111.1;
  int commandRX = 9;
  float ROLL;
  float PITCH;
  float YAW;
} gpsData;

Adafruit_Sensor_Calibration_EEPROM cal;

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

void setup()
{
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
  delay(100);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  driver.setTxPower(20, false);
  if (!driver.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    //    while (1);
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

// Dont put this on the stack:
//uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop()
{
  
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO) {
    if (c) Serial.print(c);
  }
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //  Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  aqAHRS();
      if (((abs(gpsData.ROLL) >= 80) || (abs(gpsData.PITCH) >= 80)) & deployCheck == 0 & sysCheck == 1 ) {
        gpsData.commandRX = 3;
        deployCheck = 1;
      }

  if (manager.available()) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      memmove((uint8_t*)&gpsData, buf, sizeof(gpsData));
      Serial.print("Receiving: "); Serial.println(gpsData.commandRX);

    }
    else
    {
      Serial.println("No reply, is rf95_reliable_datagram_server running?");
    }
  }
  Commands();
  sendGPS();

}

void Commands() {
  switch (gpsData.commandRX) {

    case 0: {
        digitalWrite(LED, LOW);
        digitalWrite(GP17, LOW);
        break;
      }

    case 1: {
        if (sysCheck == 0) {
          Serial.println("Check Complete");
          LED_Switch = 1;
          sysCheck = 1;
          sendcycle = 400;
          counter = 0;
        }
        digitalWrite(LED, HIGH);
        break;
      }
    case 2: {
        if (ignitCheck == 0) {
          Serial.println("Ignition");
          LED_Switch = 1;
          ignitCheck = 1;
          counter = 0;
        }

        //int ignitCheck = 1;
        digitalWrite(LED, HIGH);
        LED_Switch = 1;
        break;
      }
    case 3: {
        if (deployCheck == 0) {
          Serial.println("Deployment");
          LED_Switch = 1;
          deployCheck = 1;
          counter = 0;
        }
        digitalWrite(GP17, HIGH);
        digitalWrite(LED, HIGH);
        LED_Switch = 1;
        break;
      }
    case 9: {
        sysCheck = 0;
        ignitCheck = 0;
        deployCheck = 0;
        sendcycle = 1500;
        break;
      }
    default: {
        //int LED_Switch = 0;
        break;
      }
  }
}

void aqAHRS() {
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis() - timestamp); Serial.println(" ms");
#endif

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.z * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.y * -SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz,
                accel.acceleration.x, accel.acceleration.z, - accel.acceleration.y,
                mag.magnetic.x, mag.magnetic.z, - mag.magnetic.y);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Update took "); Serial.print(millis() - timestamp); Serial.println(" ms");
#endif

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(-accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.print(", ");
  Serial.print(-mag.magnetic.y, 4); Serial.println("");
#endif

  gpsData.ROLL = abs(filter.getRoll());
  gpsData.PITCH = abs(filter.getPitch());
  gpsData.YAW = filter.getYaw();

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Took "); Serial.print(millis() - timestamp); Serial.println(" ms");
#endif
}

void sendGPS() {

  // TimerA    approximately 1 second
  if (millis() - timer > sendcycle) {
    digitalWrite(LED, HIGH);
    gpsData.latitudeGPS = GPS.latitude;
    gpsData.longitudeGPS = GPS.longitude;
    gpsData.latGPS = GPS.lat;
    gpsData.lonGPS = GPS.lon;
    gpsData.altitudeGPS = GPS.altitude;
    if (counter == 3) {
      gpsData.commandRX = 0;
      counter = 0;
      LED_Switch = 0;
    }
    if (gpsData.commandRX != 0) {
      counter = counter + 1;
    }
    if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(buf), BROADCAST_ADDRESS)) {
      //  if (rf95.send((uint8_t*)&gpsData, sizeof(gpsData))){
      Serial.println("Sending");
      //      rf95.waitPacketSent();
      //
    }
    else
    {
      Serial.println("No reply, is server running?");
    }



    //delay(10);
    if (GPS.fix) {
      // print out the current stats
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //     Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Command: "); Serial.println(gpsData.commandRX);
      Serial.print("Pitch: "); Serial.println(gpsData.PITCH);
      Serial.print("Roll: "); Serial.println(gpsData.ROLL);
      Serial.print("Yaw: "); Serial.println(gpsData.YAW);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.println("");
    }



    timer = millis();
    digitalWrite(LED, LOW);
  }
}
