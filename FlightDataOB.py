from os import system, name
#from time import sleep
import serial
import time
import board
#import digitalio
import struct
import pandas as pd
port = serial.Serial("/dev/ttyS0", baudrate=115200)
count = 0
prevrow = 0
row = 0
gpsd = pd.DataFrame(data=[],index=[], columns=['Latitude','Lat','Longitude','Lon','Altitude','CommandTX','Roll','Pitch','Yaw'])
def clear ():
    if name == 'nt':
        _ = system('cls')
    else:
        _ = system('clear')
while True:
    try:       # if count == 0:
            RFID = port.read_until()
            latGPS = RFID.decode()
            gpsd.at[row, 'Latitude'] = latGPS[:-1]
       # elif count == 1 :
            RFID = port.read_until()
            longGPS = RFID.decode()
            gpsd.at[row, 'Longitude'] = longGPS[:-1]
  #          count = count + 1
        #elif count == 2:
            RFID = port.read_until()
            laGPS = str(RFID, 'utf-8')
            gpsd.at[row, 'Lat'] = laGPS[:-1]
 #           count = count + 1
       # elif count == 3:
            RFID = port.read_until()
            lonGPS = str(RFID, 'utf-8')
            gpsd.at[row, 'Lon'] = lonGPS[:-1]
#            count = count + 1
        #elif count == 4:
            RFID = port.read_until()
            altGPS = RFID.decode()
            gpsd.at[row, 'Altitude'] = altGPS[:-1]
    #        count = count + 1
        #elif count == 5:
            RFID = port.read_until()
            cmdTX = RFID.decode()
            gpsd.at[row, 'CommandTX'] = cmdTX[:-1]
            RFID = port.read_until()
            roll = RFID.decode()
            gpsd.at[row, 'Roll'] = roll[:-1]
            RFID = port.read_until()
            pitch = RFID.decode()
            gpsd.at[row, 'Pitch'] = pitch[:-1]
            RFID = port.read_until()
            yaw = RFID.decode()
            gpsd.at[row, 'Yaw'] = yaw[:-1]
#        if gpsd.at[row, 'CommandTX']== 1:
#            print('Check Completed')
#        elif gpsd.at[row, 'CommandTX'] == 2:
#            print('Ignition')
            prevrow = row
     #       count = 0
            row =row + 1  # print (longGPS)
            gpsd.to_csv('~/Patience/FlightData/Flightdata.csv')
            #print(gpsd)
            if gpsd.at[prevrow, 'CommandTX'] == '1':
                print("Check Completed")
            elif gpsd.at[prevrow, 'CommandTX'] == '2':
                print("Ignition")
            elif gpsd.at[prevrow, 'CommandTX'] == '3':
                print("Deployment")
            else:
                clear()
                print(gpsd.iloc[prevrow])
       # else:
           # time.sleep(0.1)
#            serialIn = raw_input()
#            if port.in_waiting > 0 :
 #               serialIn = 1 #port.read(port.in_waiting)
  #              port.write(serialIn)
    except:
        raise TypeError("error")
