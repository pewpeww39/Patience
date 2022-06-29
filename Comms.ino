

//byte gpsdata_buf[sizeof(gpsData)] = {4};

//uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
void sendGPS() {
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

  // TimerA    approximately 1 second
  if (millis() - timer > sendcycle) {
    digitalWrite(LED, HIGH);
    gpsData.latitudeGPS = GPS.latitude;
    gpsData.longitudeGPS = GPS.longitude;
    gpsData.latGPS = GPS.lat;
    gpsData.lonGPS = GPS.lon;
    gpsData.altitudeVEC = GPS.altitude;
    if (counter == 5) {
      gpsData.commandRX = 0;
      counter = 0;
      LED_Switch = 0;
    }
    if (gpsData.commandRX != 0) {
      counter = counter + 1;
    }
    if (manager.sendtoWait((uint8_t*)&gpsData, sizeof(gpsData), BROADCAST_ADDRESS)) {
   //  if (rf95.send((uint8_t*)&gpsData, sizeof(gpsData))){
      Serial.println("Sending");
//      rf95.waitPacketSent();
//
    }
    else
    {
      Serial.println("No reply, is server running?");
    }


      //delay(100);
    if (GPS.fix && debug == true) {
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
  }
}
