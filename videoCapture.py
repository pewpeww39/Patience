import os
import RPi.GPIO as GPIO


inputPin = 26
outputPin = 19

GPIO.setmode(GPIO.BOARD)
GPIO.setup(inputPin, GPIO.IN)
GPIO.setup(outputPin, GPIO.OUT)

if inputPin == 1
	os.system("libcamera-vid -t 14000 -o test.h264")
	outputPin = 1

