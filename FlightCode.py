# Create an IMU for pitch roll and yaw

import time
import board
import adafruit_fxas21002c
import adafruit_fxos8700
import digitalio
import busio

i2c = busio.I2C(board.GP5, board.GP4) #create i2c interface on 4(SDL) and 5(SCL)
fxas = adafruit_fxas21002c.FXAS21002C(i2c)
fxos = adafruit_fxos8799c.FXOS8700C(i2c)

def gyroRead():
	gyro_x, gyro_y, gyro_z = sensor.gyroscope
	return gyro_x, gyro_y, gyro_z
def accelRead():
	accel_x, accel_y, accel_z = fxos.accelerometer
	return accel_x, accel_y, accel_z
def magRead():
	mag_x, mag_y, mag_z = fxos.magnetometer
	return mag_x, mag_y, mag_Z

while True:
    #gyro_x, gyro_y, gyro_z = gyroRead()
    gyroRead()
    accelRead()
    magRead()
    print("Gyroscope (radians/s): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(gyro_x, gyro_y, gyro_z))
    print("Accelerometer (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(accel_x))
    time.sleep(1.0)

