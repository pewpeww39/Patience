#!/usr/bin/env python3
import serial
import random

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    ser.reset_input_buffer()

    while True:
        number = ser.readline().decode('utf-8').rstrip()
        wrote = bytes('wrote', 'utf-8')
        ser.write(wrote)
        if number is not None:
            print(number)
            if number  == 18: # int.from_bytes(number, byteorder='big') == 18:
                led_number = random.randint(1,4)
                print("Button has been pressed.")
                #print("Sending number " + str(led_number) + " to Arduino.")
                #ser.write(str(led_number).encode('utf-8'))
