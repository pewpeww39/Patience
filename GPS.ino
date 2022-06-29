//#include <Adafruit_GPS.h>
//
//
//void runGPS() {
//  char c = GPS.read();
//  // if you want to debug, this is a good time to do it!
//  if (GPSECHO) {
//    if (c) Serial.print(c);
//  }
//  // if a sentence is received, we can check the checksum, parse it...
//  if (GPS.newNMEAreceived()) {
//    // a tricky thing here is if we print the NMEA sentence, or data
//    // we end up not listening and catching other sentences!
//    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
//    //  Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
//    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
//      return; // we can fail to parse a sentence in which case we should just wait for another
//  }
//
//
//  // approximately every 2 seconds or so, print out the current stats
//  if (millis() - timer > 1500) {
//    timer = millis(); // reset the timer
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      //     Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//      Serial.println("");
//    }
//      }
//    //  gpsData.latitudeGPS = GPS.latitude;
//    //  gpsData.longitudeGPS = GPS.longitude;
//  }
