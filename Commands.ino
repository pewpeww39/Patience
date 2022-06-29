
void Commands() {
  switch (gpsData.commandRX) {

    case 0: {
        break;
      }

    case 1: {
        if (sysCheck == 0) {
          Serial.println("Check Complete");
          LED_Switch = 1;
          sysCheck = 1;
          sendcycle = 400;
        }
        digitalWrite(LED, HIGH);
        break;
      }
    case 2: {
        if (ignitCheck == 0) {
          Serial.println("Ignition");
          LED_Switch = 1;
          ignitCheck = 1;
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
        }
        int deployCheck = 1;
        digitalWrite(LED, HIGH);
        LED_Switch = 1;
        break;
      }
    case 9: {
      sysCheck = 0;
      ignitCheck = 0;
      deployCheck = 0;
      sendcycle = 1330;
    }
    default: {
        //int LED_Switch = 0;
        break;
      }
  }
}
