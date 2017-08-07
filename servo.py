#!/usr/bin/env python


import serial
import time
# Assign Arduino's serial port address
#   Windows example
#     usbport = 'COM3'
#   Linux example
usbport = '/dev/ttyACM0'
#   MacOSX example
#     usbport = '/dev/tty.usbserial-FTALLOK2'
#usbport = '/dev/tty.usbserial-FTALLOK2'

# Set up serial baud rate
ser = serial.Serial(usbport, 9600, timeout=1)

def move(servo, angle):

    angle = abs(angle)  # this line should be fixed later to get the right angles
    if (0 <= angle <= 180):
        ser.write(chr(255))
        ser.write(chr(servo))
        ser.write(chr(angle))
        ser.read()
    else:
        print "Servo angle must be an integer between 0 and 180.\n"
        print ser.readline()
# example
#for a in range(0,180,1):
#    move(1, a)
#for a in range(0,180,1):
#    move(2, a)