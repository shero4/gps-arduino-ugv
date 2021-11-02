import serial
import time

rfd900 = serial.Serial(port="/dev/tty.usbserial-1110", baudrate=57600, timeout=1)
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
        elif(command == '11'):
            print("GPS Info")
            rfd900.write(bytes('11\n', 'utf-8'))
            data = (rfd900.readline()).decode('utf-8')
            print(data)
    
writeRandomData()