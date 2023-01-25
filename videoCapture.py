import os
import RPi.GPIO as GPIO


inputPin = 26
outputPin = 19

GPIO.setmode(GPIO.BOARD)
GPIO.setup(inputPin, GPIO.IN)
GPIO.setup(outputPin, GPIO.OUT)
while(1):
	if inputPin == 1:
		os.system("libcamera-vid -t 60000 -o firstFlight.h264")
		outputPin = 1

