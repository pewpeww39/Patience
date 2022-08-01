from os import system, name
from time import sleep
import serial
import time
import struct
import pandas as pd
due = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout = 0.1)
def clear ():
    if name == 'nt':
        _ = system('cls')
    else:
        _ = system('clear')
def write_read(x):
    due.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    #data = arduino.readline()
    #return x #data
while True:
#    try:
        serialin =input('Enter a command: ')
        value = write_read(serialin)
#        print(value)
        time.sleep(.5)
        clear()
        # port.write(bytes(serialin, 'utf-8'))
#    except:
 #       print('something is wrong')
