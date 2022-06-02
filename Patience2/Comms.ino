

//byte gpsdata_buf[sizeof(gpsData)] = {4};

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
  if (millis() - timer > 1330) {
    digitalWrite(LED, HIGH);
    timer = millis();
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
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.println("");
    }

    gpsData.latitudeGPS = GPS.latitude;
    gpsData.longitudeGPS = GPS.longitude;
    gpsData.latGPS = GPS.lat;
    gpsData.lonGPS = GPS.lon;
    gpsData.altitudeVEC = GPS.altitude;
    if (counter == 3) {
      gpsData.commandRX = 0;
      counter = 0;
      LED_Switch = 0;
    }
    counter ++;
    rf95.send((uint8_t *)&gpsData, sizeof(gpsData));
    rf95.waitPacketSent();
    Serial.println("Sending");
    //  break;

    //case 1: {

    //return (gpsData);
    // break;


  }

}


//    recvCMD();


//void recvCMD () {
//  if (rf95.available()) {
//    // Should be a message now
//    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {};
//    uint8_t len1 = sizeof(buf);
//    if (rf95.recv(buf, &len1))
//    {
//      if (strcmp((char*)buf, "0") == 0) {
//        counter = 0;
//      }
//      else if (strcmp((char*)buf, "1") == 1)
//      {
//        counter = 1;
//      }
//      else if (strcmp((char*)buf, "2") == 0)
//      {
//        counter = 2;
//      }
//      else if (strcmp((char*)buf, "3") == 0)
//      {
//        counter = 3;
//      }
//      else {
//        counter = 0;
//      }
//      //counter = commandbuf;
//      switch (counter) {
//        case 0: {
//            uint8_t mesg[] = "Reset";
//            rf95.send(mesg, sizeof(mesg));
//            rf95.waitPacketSent();
//            digitalWrite(LED, HIGH);
//            //digitalWrite(LED, LOW);
//          }
//        case 1: {
//            uint8_t mesg[] = "ol";
//            rf95.send(mesg, sizeof(mesg));
//            rf95.waitPacketSent();
//            digitalWrite(LED, HIGH);
//            delay(2500);
//          }
//        case 2: {
//            uint8_t mesg[] = "Ignition";
//            rf95.send(mesg, sizeof(mesg));
//            rf95.waitPacketSent();
//            digitalWrite(LED, HIGH);
//          }
//
//      }
//    }
//
//  }
//}
