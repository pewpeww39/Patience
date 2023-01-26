import os
import RPi.GPIO as GPIO


inputPin = 6
outputPin = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(outputPin, GPIO.OUT)
GPIO.setup(inputPin, GPIO.IN, initial=1)

while(1):
        if GPIO.input(inputPin) == 1:
                os.system("libcamera-vid -t 14000 -o test.h264 -b 10000000 --wi>
                outputPin = 1
                break;

