
void Commands() {
  switch (gpsData.commandRX) {

    case 0: {
        break;
      }

    case 1: {
        Serial.print("Check Complete");

        digitalWrite(LED, HIGH);
        LED_Switch = 1;
        break;
      }
    case 2: {
        Serial.print("Ignition");

        digitalWrite(LED, HIGH);
        LED_Switch = 1;
        break;
      }
    case 3: {
        Serial.print("Deployment");

        digitalWrite(LED, HIGH);
        LED_Switch = 1;
        break;
      }
    default: {
        //int LED_Switch = 0;
        break;
      }
  }
}
