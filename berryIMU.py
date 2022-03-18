    #!/usr/bin/python
    #
    #       This program includes a number of calculations to improve the
    #       values returned from a BerryIMU. If this is new to you, it
    #       may be worthwhile first to look at berryIMU-simple.py, which
    #       has a much more simplified version of code which is easier
    #       to read.
    #
    #
    #       The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported
    #
    #       This script is python 2.7 and 3 compatible
    #
    #       Feel free to do whatever you like with this code.
    #       Distributed as-is; no warranty is given.
    #
    #       https://ozzmaker.com/berryimu/
from picamera import PiCamera
import time
import math
import board
import IMU
import datetime
import os
import sys
import RPi.GPIO as GPIO
import busio
from digitalio import DigitalInOut, Direction, Pull
import adafruit_ssd1306
import adafruit_rfm9x
global beta
global counter 
global sent 
global command2Sent 
global command3Send 
global command4Sent
counter = 0
sent = 0
command2Sent = 0
command3Send = 0
command4Sent = 0
beta = 0

CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0, baudrate=115200)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0, baudrate=9600)
rfm9x.tx_power = 23
prev_packet = None
IMU.detectIMU()     #Detect if BerryIMU is connected.
if(IMU.BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
rfm9x.send(bytes('Communications online                 \r','utf-8'))

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
kalmanX = 0.0
kalmanY = 0.0

#camera=PiCamera ()
#time.sleep (2)
#camera.resolution = (1280, 720)
#camera.vflip = True
#camera.contrast = 10
#file_name = "/home/pi/Patience/video_" + str(time.time()) + ".h264"
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant

GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT, initial=0)
GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.OUT, initial = 0)

print("Waiting to connect...")
#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0

def kalmanFilterX ( accAngle, gyroRate, DT):
    x=0.0
    S=0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    global XP_00
    global XP_01
    global XP_10
    global XP_11


    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
    XP_01 = XP_01 + ( - DT * XP_11 )
    XP_10 = XP_10 + ( - DT * XP_11 )
    XP_11 = XP_11 + ( + Q_gyro * DT )

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + ( K_0 * x )
    x_bias = x_bias + ( K_1 * x )

    XP_00 = XP_00 - ( K_0 * XP_00 )
    XP_01 = XP_01 - ( K_0 * XP_01 )
    XP_10 = XP_10 - ( K_1 * XP_00 )
    XP_11 = XP_11 - ( K_1 * XP_01 )

    return KFangleX

def kalmanFilterY ( accAngle, gyroRate, DT):
    y=0.0
    S=0.0

    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
    YP_01 = YP_01 + ( - DT * YP_11 )
    YP_10 = YP_10 + ( - DT * YP_11 )
    YP_11 = YP_11 + ( + Q_gyro * DT )

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + ( K_0 * y )
    y_bias = y_bias + ( K_1 * y )

    YP_00 = YP_00 - ( K_0 * YP_00 )
    YP_01 = YP_01 - ( K_0 * YP_01 )
    YP_10 = YP_10 - ( K_1 * YP_00 )
    YP_11 = YP_11 - ( K_1 * YP_01 )

    return KFangleY


def Patience(RFsignal):
#    from picamera import PiCamera
#    import time
#    import math
#    import board
#    import IMU
#    import datetime
#    import os
#    import sys
#    import RPi.GPIO as GPIO
#    import busio
#    from digitalio import DigitalInOut, Direction, Pull
#    import adafruit_ssd1306
#    import adafruit_rfm9x

    # Create the I2C interface.

    ################# Compass Calibration values ############
    # Use calibrateBerryIMU.py to get calibration values
    # Calibrating the compass isnt mandatory, however a calibrated
    # compass will result in a more accurate heading value.

    magXmin =  399
    magYmin =  -3285
    magZmin =  1870
    magXmax =  488
    magYmax =  -3224
    magZmax =  1971


    '''
    Here is an example:
    magXmin =  -1748
    magYmin =  -1025
    magZmin =  -1876
    magXmax =  959
    magYmax =  1651
    magZmax =  708
    Dont use the above values, these are just an example.
    '''
    ############### END Calibration offsets #################
    global counter
    global sent
    global command2Sent
    global command3Send
    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0
    kalmanX = 0.0
    kalmanY = 0.0

    

    a = datetime.datetime.now()

    #print("Start recording...")
    #camera.start_recording(file_name)
    #camera.wait_recording(5)
    #camera.stop_recording()
    global beta 

#    while True:
    blastoff = RFsignal
    if blastoff is not None:
        packet_text = str(blastoff, "utf-8")
        print("Received (utf-8): {0}".format(packet_text))
        counter = counter + 1

    if ((counter == 1) and blastoff is not None):
        if (sent == 0):
            sent = 1
            rfm9x.send(bytes('Ready for takeoff                      \r', 'utf-8'))
            
    if (counter == 2):
        if (command2Sent == 0):
            command2Sent = 1
            rfm9x.send(bytes('TAKEOFF!                              \r', 'utf-8'))
            GPIO.output(24,1)
            time.sleep(01.50)
        else:
            GPIO.output(24,0)
    if (counter == 3):
        if (command3Send == 0):
            command3Send = 1
            rfm9x.send(bytes('Deployment!                           \r', 'utf-8'))
        if ((command3Send <= 5) and (beta <= 5)):
            GPIO.output(26,1)
            command3Send += 1
            beta += 1
           # time.sleep(01.50)
    if (counter == 4):
#        if (command4Sent == 0):
#            command4Sent == 1
        counter = 0
        beta = 0
        sent = 0
        command2Sent = 0
        command3Send = 0
        command4Sent = 0 
        rfm9x.send(bytes('Reset', 'utf-8'))
    packet_text = None
    blastoff = None
#            command3Sent = 1 #make elseif so that primary deploy calls this case
#Read the accelerometer,gyroscope and magnetometer values
    ACCx = IMU.readACCz()
    ACCy = IMU.readACCx()
    ACCz = IMU.readACCy()
    GYRx = IMU.readGYRz()
    GYRy = IMU.readGYRx()
    GYRz = IMU.readGYRy()
    MAGx = IMU.readMAGz()
    MAGy = IMU.readMAGx()
    MAGz = IMU.readMAGy()


    #Apply compass calibration
    MAGx -= (magZmin + magZmax) /2
    MAGy -= (magXmin + magXmax) /2
    MAGz -= (magYmin + magYmax) /2


    ##Calculate loop Period(LP). How long between Gyro Reads
    b = datetime.datetime.now() - a
    a = datetime.datetime.now()
    LP = b.microseconds/(1000000*1.0)
    outputString = "Loop Time %5.2f " % ( LP )

    

    #Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  -GYRy * G_GAIN
    rate_gyr_z =  -GYRz * G_GAIN


    #Calculate the angles from the gyro.
    gyroXangle+=rate_gyr_x*LP
    gyroYangle+=rate_gyr_y*LP
    gyroZangle+=rate_gyr_z*LP



   #Convert Accelerometer values to degrees
    AccYangle =  -(math.atan2(ACCy,ACCz)*RAD_TO_DEG) + 180
    AccXangle =  -(math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG +90
    AccZangle =  (math.atan2(ACCy,ACCx)*RAD_TO_DEG)
   #convert the values to -180 and +180
    if AccXangle > 180:
        AccXangle -= 360.0
    if AccYangle > 180 or AccYangle < 0:
        AccYangle -= 360
        
    #else:
        #AccYangle += 90.0


    #Complementary filter used to combine the accelerometer and gyro values.
    CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
    CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

    #Kalman filter used to combine the accelerometer and gyro values.
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)


    #Calculate heading
    heading = 180 * math.atan2(MAGy,MAGx)/M_PI

    #Only have our heading between 0 and 360
    if heading < 0:
        heading += 360





    ####################################################################
    ###################Tilt compensated heading#########################
    ####################################################################
    #Normalize accelerometer raw values.
    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


    #Calculate pitch and roll
    pitch = math.asin(accXnorm)
    try:
        roll = -math.asin(accYnorm/math.cos(pitch))
    except Exception:
        roll = 1


    #Calculate the new tilt compensated values
    #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
    #This needs to be taken into consideration when performing the calculations

    #X compensation
    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
    else:                                                                #LSM9DS1
        magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

    #Y compensation
    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
    else:                                                                #LSM9DS1
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)




    #Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

    if tiltCompensatedHeading < 0:
        tiltCompensatedHeading += 360


    ##################### END Tilt Compensation ########################


    if 1:                       #Change to '0' to stop showing the angles from the accelerometer
        outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f ACCZ Angle %5.2f #  " % (AccXangle, AccYangle, AccZangle)

    if 0:                       #Change to '0' to stop  showing the angles from the gyro
        outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)

    if 0:                       #Change to '0' to stop  showing the angles from the complementary filter
        outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)

    if 0:                       #Change to '0' to stop  showing the heading
        outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)

    if 0:                       #Change to '0' to stop  showing the angles from the Kalman filter
        outputString +="\t# kalmanX %5.2f   kalmanY %5.2f #" % (kalmanX,kalmanY)
    if 1:
        outputString +="\t# Pitch %5.2f Roll %5.2f beta %5.2f counter %5.2f #" % (pitch, roll, counter, beta)
    if (abs(pitch)>=1.3 or abs(roll)>=1.3) and beta == 0:
        outputString +="\n Ignition"
        counter = 3
        command3Send = 0
        #GPIO.output(26, 1)
        #strPitch=str(pitch)
        #GPIO.output(26, 1)
#        data = bytearray('Deployed ', 'utf-8')
#        rfm9x.send(data)
        beta = 1
#        time.sleep(2)
    elif (abs(pitch) >= 1.3 or abs(roll)>=1.3) and beta >= 5:
        GPIO.output(26, 0)
    print(outputString)
    #slow program down a bit, makes the output more readable
#    time.sleep(0.03)
    #camera.stop_recording()
    i2c = busio.I2C(board.SCL, board.SDA)
init = 0
while True:
    Comm = rfm9x.receive()
    if Comm is not None:
        Patience(Comm)
        init = 1
    elif ((Comm is None) and (init != 1)):
        init = 0 
    else:
        Patience(None)
            
