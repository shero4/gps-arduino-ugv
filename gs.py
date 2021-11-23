#! /usr/bin/python3

import threading
import serial
import time
import sys

ttyPort = "/dev" + str(sys.argv[1])

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

def getTelemetry():
    while True:
        if(rfd900.inWaiting() > 0):
            try:
                telem = rfd900.readline().decode('utf-8')
                print(telem)
                if "#" in telem:
                    telem = telem.split("#")
                    print(telem)
                    distance_to_destination = telem[0]
                    desired_heading = telem[1]
                    ugv_heading = telem[6]
                    current_lat = telem[2]
                    current_lon = telem[3]
                    destination_lat = telem[4]
                    destination_lon = telem[5]
                    compass_heading = telem[-1]
                    print("Distance to destination: " + distance_to_destination)
                    print("Desired heading: " + desired_heading)
                    print("UGV heading: " + ugv_heading)
                    print("Current lat: " + current_lat)
                    print("Current lon: " + current_lon)
                    print("Destination lat: " + destination_lat)
                    print("Destination lon: " + destination_lon)
                    print("Compass heading: " + compass_heading)
                else:
                    print(telem)

            except:
                print("Error")

            


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
            
# manualControl()
t1 = threading.Thread(target=manualControl)
t2 = threading.Thread(target=getTelemetry)


# starting thread 1
t1.start()
# starting thread 2
t2.start()


# wait until thread 1 is completely executed
t1.join()
# wait until thread 2 is completely executed
t2.join()
