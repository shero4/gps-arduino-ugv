#include <TinyGPS++.h>
#include "Wire.h"
#include "MPU9250.h"
#include <Servo.h>

// Ultrasonic Stuff
#define trigPin1 47
#define echoPin1 46
#define trigPin2 42
#define echoPin2 43
long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor;
int count = 0;

// IMU - Magnetometer Stuff
MPU9250 mpu;
float mx, my, mz;
int desired_heading;
int compass_heading;
int compass_dev = 5;
int Heading_A;
int Heading_B;
int pass = 0;

// GPS Stuff
int GPS_Course;
int Number_of_SATS;
TinyGPSPlus gps;

// GPS Action stuff
unsigned long Distance_To_Home;
int ac = 0;
int wpCount = 0;
double Home_LATarray[50];
double Home_LONarray[50];
int increment = 0;

// RFD stuff
String str;
int rfdVal;

// Servo stuff
int servo1Pin = 10;
int servo2Pin = 11;
Servo Servo1;
Servo Servo2;

// Motor stuff
int mtr_Spd = 100;
int turn_speed = 200;
const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 7;
const int ENA = 8;
const int ENB = 9;

void getGPS()  // Get Latest GPS coordinates
{
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
}

void setWaypoint()  // Set up to 5 GPS waypoints
{

  if (wpCount >= 0) {
    Serial1.print("GPS Waypoint ");
    Serial1.print(wpCount + 1);
    Serial1.print(" Set ");
    getGPS();      // get the latest GPS coordinates
    getCompass();  // update latest compass heading

    Home_LATarray[ac] = gps.location.lat();  // store waypoint in an array
    Home_LONarray[ac] = gps.location.lng();  // store waypoint in an array

    Serial.print("Waypoint #1: ");
    Serial.print(Home_LATarray[0], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0], 6);
    Serial.print("Waypoint #2: ");
    Serial.print(Home_LATarray[1], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[1], 6);
    Serial.print("Waypoint #3: ");
    Serial.print(Home_LATarray[2], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[2], 6);
    Serial.print("Waypoint #4: ");
    Serial.print(Home_LATarray[3], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[3], 6);
    Serial.print("Waypoint #5: ");
    Serial.print(Home_LATarray[4], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[4], 6);

    wpCount++;  // increment waypoint counter
    ac++;       // increment array counter
  } else {
    Serial.print("Waypoints Full");
  }
}

void Startup() {
  Serial.println("Searching for Satellites ");
  while (Number_of_SATS <= 4)  // Wait until x number of satellites are acquired before starting main loop
  {
    getGPS();                                        // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());  // Query Tiny GPS for the number of Satellites Acquired
    Serial.println(char(Number_of_SATS) + " sats");
    bluetooth();                                     // Check to see if there are any bluetooth commands being received
  }
  Serial.print(Number_of_SATS);
  Serial.println(" Satellites Acquired");
  setWaypoint();  // set intial waypoint to current location
  wpCount = 0;    // zero waypoint counter
  ac = 0;         // zero array counter
}

void setup() {
  // Motor Pins Setup
  Serial.println("Setting up Arduino");
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic Setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  //Servos Setup
  Servo1.attach(servo1Pin);
  Servo2.attach(servo2Pin);
  Servo1.write(180);
  Servo2.write(90);

  // Serial Setup
  Serial.begin(115200);   // our serial monitor
  Serial3.begin(57600); // RFD900
  Serial2.begin(38400);  // bluetooth connection
  Serial1.begin(9600);  // GPS serial bus

  Serial.println("Starting IMU");
  Wire.begin();
  delay(2000);
  mpu.setup(0x68);

  //  Serial.println("Calibrating IMU");
  //  delay(5000);
  //  mpu.calibrateAccelGyro();
  //  mpu.calibrateMag();

//  Startup();
}

void getCompass() {
  mpu.update();
  if (mpu.update()) {
    //    Serial.println(mpu.getYaw());
    // Magnetometer
    mx = mpu.getMagX();
    my = mpu.getMagY();
    mz = mpu.getMagZ();

    // set heading
    float heading = atan2(my, mx);
    if (heading < 0)
      heading += 2 * M_PI;
    compass_heading = (int)(heading * 180 / M_PI);
    compass_heading = abs(compass_heading - 360);
    compass_heading = (((int)mpu.getYaw() + 180) + 90) % 360;
    // End of line
    Serial.println(compass_heading);
    delay(100);
  }
}

// Steering //

void Ping() {}

void Forward() {
  Serial.println("Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, mtr_Spd);
  analogWrite(ENB, mtr_Spd);
}

void Reverse() {
  Serial.println("Reverse");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, mtr_Spd);
  analogWrite(ENB, mtr_Spd);
}

void Forward_Meter() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, mtr_Spd);
  analogWrite(ENB, mtr_Spd);
  delay(500);
}

void LeftTurn() {
  Serial.println("Left Turn");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, turn_speed);
  analogWrite(ENB, turn_speed);
  delay(100);
}

void RightTurn() {
  Serial.println("Right Turn");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, turn_speed);
  analogWrite(ENB, turn_speed);
  delay(100);
}

void SlowLeftTurn() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, turn_speed);
  analogWrite(ENB, turn_speed);
  delay(100);
}

void SlowRightTurn() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, turn_speed);
  analogWrite(ENB, turn_speed);
  delay(100);
}

void StopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Compass Drive Section
// This Portion of code steers the Vehicle based on the compass heading
// **********************************************************************************************************************************************************************

void CompassTurnRight()
{
  StopCar();
  getCompass();

  desired_heading = compass_heading + 90; // Right turn = compass_heading+90 degree right

  if (desired_heading >= 360) {
    desired_heading = (desired_heading - 360);
  }

  while (abs(desired_heading - compass_heading) >= compass_dev)
  {
    getCompass();
    bluetooth();
    if (rfdVal == 5) {
      break;
    }

    int z = (compass_heading - desired_heading - 1);
    Serial.print(compass_heading); Serial.print(" ");
    Serial.print(desired_heading); Serial.print(" ");
    Serial.print(z); Serial.print(" ");
    if ((z <= 180) && (z >= 0))
    {
      Serial.println("left");
      SlowLeftTurn();
    } else {
      Serial.println("right");
      SlowRightTurn();
    }
  }
  {
    StopCar();
  }
}

// **********************************************************************************************************************************************************************

void CompassTurnLeft()
{
  StopCar();
  getCompass();

  desired_heading = (compass_heading - 90);
  if (desired_heading <= 0) {
    desired_heading = (desired_heading + 360);
  }

  while (abs(desired_heading - compass_heading) >= compass_dev)
  {
    getCompass();
    bluetooth();
    if (rfdVal == 5) {
      break;
    }

    int z = (compass_heading - desired_heading - 1);
    if (z >= 180)
    {
      SlowRightTurn();
    } else {
      SlowLeftTurn();
    }
  }
  {
    StopCar();
  }
}

// **********************************************************************************************************************************************************************

void Compass_Forward() {
  while (rfdVal == 9)  // Go forward until Bluetooth 'Stop' command is sent

    //while (true)
  {
    getCompass();  // Update Compass Heading
    bluetooth();   // Check to see if any Bluetooth commands have been sent
    if (rfdVal == 5) {
      break;
    }  // If a Stop Bluetooth command ('5') is received then break from the Loop

    if (abs(desired_heading - compass_heading) <= compass_dev)  // If the Desired Heading and the Compass Heading are within the compass deviation, X degrees of each other then Go Forward
      // otherwise find the shortest turn radius and turn left or right
    {
      Forward();
      Ping();
    } else {
      int x = (desired_heading - 359);  // x = the GPS desired heading - 360
      int y = (compass_heading - (x));  // y = the Compass heading - x
      int z = (y - 360);                // z = y - 360

      if ((z <= 180) && (z >= 0))  // if z is less than 180 and not a negative value then turn left
      { // otherwise turn right
        SlowLeftTurn();
        Ping();
      } else {
        SlowRightTurn();
        Ping();
      }
    }
  }  // End While Loop
}  // End Compass_Forward

// **********************************************************************************************************************************************************************

void turnAround()  // This procedure turns the Car around 180 degrees, every time the "Turn Around" button is pressed
{ // the car alternates whether the next turn will be to the left or right - this is determined by the 'pass' variable
  // Imagine you are cutting the grass, when you get to the end of the row - the first pass you are turning one way and on the next pass you turn the opposite
  if (pass == 0) {
    CompassTurnRight();
  }  // If this is the first pass then turn right

  else {
    CompassTurnLeft();
  }  // If this is the second pass then turn left

  //Forward_Meter();                                              // Go forward one meter (approximately)
  StopCar();  // Stop the car

  if (pass == 0)  // If this is the first pass then Turn Right
  {
    CompassTurnRight();  // Turn right
    pass = 1;            // Change the pass value to '1' so that the next turn will be to the left
  }

  else {

    if (desired_heading == Heading_A)  // This section of code Alternates the desired heading 180 degrees
    { // for the Compass drive forward
      desired_heading = Heading_B;
    } else if (desired_heading == Heading_B) {
      desired_heading = Heading_A;
    }

    CompassTurnLeft();  // If this is the second pass then Turn Left
    pass = 0;           // Change the pass value to '0' so that the next turn will be to the right
  }

  Compass_Forward();  // Maintain the 'desired heading' and drive forward
}

void bluetooth() {
  while (Serial3.available())  // Read bluetooth commands over Serial2 // Warning: If an error with Serial2 occurs, make sure Arduino Mega 2560 is Selected
  {
    Serial.print("YO");
    {
      str = Serial3.readStringUntil('\n');
    }
    rfdVal = (str.toInt());  //  convert the string 'str' into an integer and assign it to rfdVal
    Serial.print("RFD Value ");
    Serial.println(rfdVal);

    switch (rfdVal) {
      case 1:
        Serial2.println("Forward");
        Forward();
        break;

      case 2:
        Serial2.println("Reverse");
        Reverse();
        break;

      case 3:
        Serial2.println("Left");
        LeftTurn();
        StopCar();
        break;

      case 4:
        Serial2.println("Right");
        RightTurn();
        StopCar();
        break;

      case 5:
        Serial2.println("Stop Car ");
        StopCar();
        break;

      case 6:
        setWaypoint();
        break;

      case 7:
        goWaypoint();
        break;

      case 8:
        Serial2.println("Turn Around");
        turnAround();
        break;

      case 9:
        Serial2.println("Compass Forward");
        setHeading();
        Compass_Forward();
        break;

      case 10:
        setHeading();
        break;

      case 11:
        gpsInfo();
        break;

      case 12:
        Serial2.println("Compass Turn Right");
        CompassTurnRight();
        break;

      case 13:
        Serial2.println("Compass Turn Left");
        CompassTurnLeft();
        break;

      case 14:
        Serial2.println("Calibrate Compass");
        // calibrateCompass();
        break;

      case 15:
        // pingToggle();
        break;

      case 16:
        // clearWaypoints();
        break;

      case 17:  // finish with waypoints
        ac = 0;
        Serial2.print("Waypoints Complete");
        break;


    }  // end of switch case

    // **************************************************************************************************************************************************
    // Slider Value for Speed

    if (rfdVal) {
      if (rfdVal >= 1000) {
        Serial2.print("Speed set To:  ");
        Serial2.println(rfdVal - 1000);
        turn_speed = (rfdVal - 1000);
        Serial.println();
        Serial.print("Turn Speed ");
        Serial.println(rfdVal);
      }
    }
  }  // end of while loop Serial2 read

  // if no data from Bluetooth
  if (Serial2.available() == 0) {
    //    Serial.println("Bluetooth not connected");
  }
  if (Serial2.available() < 0)  // if an error occurs, confirm that the arduino mega board is selected in the Tools Menu
  {
    Serial2.println("No Bluetooth Data ");
  }

}  // end of bluetooth procedure

void gpsInfo()  // displays Satellite data to user
{
  Number_of_SATS = (int)(gps.satellites.value());                                                                                 //Query Tiny GPS for the number of Satellites Acquired
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
  Serial3.print("Lat:");
  Serial3.print(gps.location.lat(), 6);
  Serial3.print(" Lon:");
  Serial3.print(gps.location.lng(), 6);
  Serial3.print(" ");
  Serial3.print(Number_of_SATS);
  Serial3.print(" SATs ");
  Serial3.print(Distance_To_Home);
  Serial3.print("m");
  Serial.print("Distance to Home ");
  Serial.println(Distance_To_Home);
}

void setHeading()
// This procedure will set the current heading and the Heading(s) of the robot going away and returning using opposing degrees from 0 to 360;
// for instance, if the car is leaving on a 0 degree path (North), it will return on a 180 degree path (South)
{
  for (int i = 0; i <= 5; i++)  // Take several readings from the compass to insure accuracy
  {
    getCompass();  // get the current compass heading
  }

  desired_heading = compass_heading;  // set the desired heading to equal the current compass heading
  Heading_A = compass_heading;        // Set Heading A to current compass
  Heading_B = compass_heading + 180;  // Set Heading B to current compass heading + 180

  if (Heading_B >= 360)  // if the heading is greater than 360 then subtract 360 from it, heading must be between 0 and 360
  {
    Heading_B = Heading_B - 360;
  }

  Serial2.print("Compass Heading Set: ");
  Serial2.print(compass_heading);
  Serial2.print(" Degrees");

  Serial.print("desired heading");
  Serial.println(desired_heading);
  Serial.print("compass heading");
  Serial.println(compass_heading);
}

void goWaypoint() {
  Serial2.println("Go to Waypoint");

  while (true) {                       // Start of Go_Home procedure
    bluetooth();                       // Run the Bluetooth procedure to see if there is any data being sent via BT
    if (rfdVal == 5) {
      break;  // If a 'Stop' Bluetooth command is received then break from the Loop
    }
    getCompass();                      // Update Compass heading
    getGPS();                          // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10

    if (millis() > 5000 && gps.charsProcessed() < 10)  // If no Data from GPS within 5 seconds then send error
      Serial2.println(F("No GPS data: check wiring"));

    Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);               //Query Tiny GPS for Course to Destination
    if (Distance_To_Home == 0)                                                                                                      // If the Vehicle has reached it's Destination, then Stop
    {
      StopCar();                              // Stop the robot after each waypoint is reached
      Serial2.println("You have arrived!");  // Print to Bluetooth device - "You have arrived"
      ac++;                                   // increment counter for next waypoint
      break;                                  // Break from Go_Home procedure and send control back to the Void Loop
      // go to next waypoint
    }


    if (abs(GPS_Course - compass_heading) <= 15)  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward
      // otherwise find the shortest turn radius and turn left or right
    {
      Forward();  // Go Forward
    } else {
      int x = (GPS_Course - 360);       // x = the GPS desired heading - 360
      int y = (compass_heading - (x));  // y = the Compass heading - x
      int z = (y - 360);                // z = y - 360

      if ((z <= 180) && (z >= 0))  // if z is less than 180 and not a negative value then turn left otherwise turn right
      {
        SlowLeftTurn();
      } else {
        SlowRightTurn();
      }
    }


  }  // End of While Loop


}  // End of Go_Home procedure

void SonarSensor(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
}

bool ultrasonic() {
  SonarSensor(trigPin1, echoPin1);
  RightSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  LeftSensor = distance;

  // Send data to RFD
  Serial3.print(LeftSensor);
  Serial3.print(" - ");
  Serial3.println(RightSensor);
  
  // Print data on serial
  Serial.print(LeftSensor);
  Serial.print(" - ");
  Serial.println(RightSensor);
  int prevLeft = 7, prevRight = 7;

  if (prevLeft >= LeftSensor && prevRight >= RightSensor && prevLeft <= 7 && prevRight <= 7 ) {
    prevLeft = LeftSensor;
    prevRight = RightSensor;
    count++;
  } else {
    count = 0;
  }
  Serial.println(count);
  if (count == 6) {
    Serial3.print("$9\n");
    Serial.println("UGV Landed");
    return true;
  } else {
    return false;
  }
}

bool ugvDropped = false;
bool flag = false;
int a = 0;

void loop() {
  if(!flag) {
    ugvDropped = ultrasonic();
    delay(500);
  }
  if(ugvDropped == true) {flag = true;}
  if(flag == true) {
    if(a == 0) {
      // only runs once
      Serial.println("Unhooking winch");
      Serial.println(a);
      Servo1.write(0);
      Servo2.write(180);
      delay(1000);
      Servo2.write(90);
      a = 1;
    }
    bluetooth();
    getGPS();
    getCompass();
  } else {
    Serial.println("Waiting for UGV to drop");
  }
}
