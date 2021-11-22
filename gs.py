#! /usr/bin/python3

import serial
import time
import sys

ttyPort = "/dev/" + str(sys.argv[1])

rfd900 = serial.Serial(port=ttyPort, baudrate=57600, timeout=1)
time.sleep(3)

def winchDrop():
    while True:
        data = rfd900.readline()
        if(data):
            print(data.decode('utf-8'))
        if data == "$9":
            # Pull Winch up
            pass

def writeRandomData():
    while True:
        rfd900.write(b'A')

def manualControl(): 
    while True:
        command = str(input())
        if(command == '1'):
            print("Forward")
            rfd900.write(bytes('1\n', 'utf-8'))
        elif(command == '2'):
            print("Backward")
            rfd900.write(bytes('2\n', 'utf-8'))
        elif(command == '3'):
            print("Left")
            rfd900.write(bytes('3\n', 'utf-8'))
        elif(command == '4'):
            print("Right")
            rfd900.write(bytes('4\n', 'utf-8'))
        elif(command == '5'):
            print("Stop Car")
            rfd900.write(bytes('5\n', 'utf-8'))
        elif(command == '6'):
            print("GPS Info")
            rfd900.write(bytes('6\n', 'utf-8'))
            data = (rfd900.readline()).decode('utf-8')
            print(data)
        elif(command == '7'):
            print("Compass Left")
            rfd900.write(bytes('7\n', 'utf-8'))
        elif(command == '8'):
            print("Compass Right")
            rfd900.write(bytes('8\n', 'utf-8'))
        elif(command == '9'):
            print("Go to waypoint")
            rfd900.write(bytes('9\n', 'utf-8'))
            while True:
                print(rfd900.readline().decode('utf-8'))
            
manualControl()
