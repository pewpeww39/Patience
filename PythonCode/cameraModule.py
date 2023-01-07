import subprocess
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.IN)
#cameraSwitch = GPIO.input(26)
cameraCh = 0
while (cameraCh == 0):
	cameraSwitch = GPIO.input(26)
	if (cameraSwitch == 1):
		print("starting video")
		cameraCh = 1
		subprocess.call('./libcamera.sh')
	else:
		print("low")
