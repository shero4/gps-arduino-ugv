# Arduino Based UGV

> Final script is in rfd_udv folder

The Arduino uses RFD900 for communication, HCSR04 Ultrasonic sensors for detecting drop, GPS Neo M6 for GPS location, MPU9250 IMU for a sense of direction, and Arduino mega to bring it all together.

## Hardware setup

The script starts with defining the ports at which the hardware devices are connected; it also defines the turn_speed(ie. speed when motor is turning) and mtr_Spd(ie. speed when motor moves forward) of the UGV

## Drop Check

Initially, the script is running a routine to check if both ultrasonic sensors are giving a value <7, if this value is received for more than 3 seconds - a message is sent to the RFD to trigger the winch pull up and servos are triggered to release the winch, after which the process of GPS guidance starts.

## IMU Setup

mpu.calibrateAccelGyro();
mpu.calibrateMag();

These functions are used to calibrate the UGV. For calibrateAccelGyro, the robot must be kept still butt for calibrateMag; you must move the robot in a '8' motion.

The MPU9250.h library handles the remaining setup.

## GPS Setup

The GPS requires around 10-20 minutes, just power it up and wait for the red light to flash. There is no indication of whether the GPS is on or not unless it connects to satellites. 

One potential way to check the number of satellites is that the Serial port is available and is encoding data using the tiny gps++ library

## getCompass() and getGPS()

Both these functions update the global variables of heading and GPS position to real-time values of the UGV, they are called(and must be called) before every critical action since the process runs on a single thread.

## Manual Control

There is a ground station script called gs.py which can be used for manual control or getting telemetry data from the UGV. It runs by a simple switch case in the Bluetooth() function. The bluetooth() function is also used to set the destination GPS location. Manual control of the robot can be taken from here by sending the appropriate commands as listed in the switch case.

## Compass Control

CompassTurnLeft and CompassTurnRight are two functions that can be used to check whether the IMU works properly and whether the UGV is following basic guidance. If you call CompassTurnLeft, the UGV should(ideally) turn left by exactly 90 degrees(+- 5 degree error). Even if it overshoots, it should oscillate between moving left or right until it does so it reach the desired heading. This *must must must* work before even trying to do GPS navigation.

## GPS Navigation

The goWaypoint() function is called after setting the heading using setHeading(), it basically runs a loop and checks
1. Whether there is a stop command via Bluetooth
2. updates the heading and GPS coordinates
3. checks if the desired heading matches the current heading
4. if yes, the robot stops. if no, it calls SlowLeftTurn or SlowRightTurn based on the current heading and desired heading values

## Problems

1. The IMU is consistently giving incorrect values and is off by 10-15 degrees
2. The Frame of reference for the IMU is at a 270 degree rotated frame of reference(can be corrected mathematically)
3. GPS location is only accurate up to a 1.5-metre radius
4. If the UGV is rotated 180 degrees, according to the IMU it has only rotated by 90 degrees? but sometimes this is fixed randomly
