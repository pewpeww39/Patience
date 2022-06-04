import serial
import time
import struct
import pandas as pd
port = serial.Serial("/dev/ttyS0", baudrate=115200)
count = 0
row = 0
gpsd = pd.DataFrame(data=[],index=[], columns=['Latitude','Lat','Longitude','Lon','Altitude','CommandTX','Pitch','Roll','Yaw'])
while True:
        if count == 0:
            RFID = port.read(11)
            latGPS = RFID.decode()
            gpsd.at[row, 'Latitude'] = latGPS
            count = count + 1
           # print (latGPS)
        elif count == 1 :
            RFID = port.read(11)
            longGPS = RFID.decode()
            gpsd.at[row, 'Longitude'] = longGPS
            count = count + 1
        elif count == 2:
            RFID = port.read(1)
            #laGPS = RFID.decode()
            gpsd.at[row, 'Lat'] = RFID
            count = count + 1
        elif count == 3:
            RFID = port.read(1)
#            lonGPS = RFID.decode()
            gpsd.at[row, 'Lon'] = RFID
            count = count + 1
        elif count == 4:
            RFID = port.read(5)
            altGPS = RFID.decode()
            gpsd.at[row, 'Altitude'] = altGPS
            count = count + 1
        elif count == 5:
            RFID = port.read(1)
            cmdTX = RFID.decode()
            gpsd.at[row, 'CommandTX'] = cmdTX
#        if gpsd.at[row, 'CommandTX']== 1:
#            print('Check Completed')
#        elif gpsd.at[row, 'CommandTX'] == 2:
#            print('Ignition')

            count = 0
            row =row + 1  # print (longGPS)
            gpsd.to_csv('/home/pi/Patience/FlightData/Flightdata.csv')
            print(gpsd)
        time.sleep (0.1)
