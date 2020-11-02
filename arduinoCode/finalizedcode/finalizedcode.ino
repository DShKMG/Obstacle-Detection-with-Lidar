
//Now we are defining the motor connections
int EnA = 8;
int In1 = 9;
int In2 = 10;

#define RPLIDAR_MOTOR 12 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal


//Here we include the servo library
#include <Servo.h>
#include <RPLidar.h>
#include <limits.h>


Servo servo;  // create servo object to control a servo
RPLidar lidar; // Create lidar object for lidar sensor

float distArray[100];
float angleArray[100];
float OldAngle = 0;
int i =0;

void setup() {

  Serial.begin (9600); //we're going to use arduino serial monitor to monitor the results, so we setup the serial monitor at 9600 baud
  lidar.begin(Serial2);

  // setup the motor control pins
  //---------------------------------------------------------
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  //---------------------------------------------------------

  servo.attach(11);  // attaches the servo on to pin 11 to the servo object

  // Print Values
  //---------------------------------------------------------
  Serial.print("\nDist");
  Serial.print("\t");
  Serial.print("Angle");
  Serial.print("\t");
  Serial.print("Servo");
  Serial.print("\t");
  Serial.print("Sample");
  Serial.print("\n");
  
}

//Now we are going to create a set of functions that will be used to determine the movement of the car

void goForward()   //run motor forward
{
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(EnA, HIGH);
}

void goBackward()   //run motor backwards
{
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(EnA, HIGH);
}

void goNothing()   //run motor backwards
{
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(EnA, HIGH);
}


void turnSides(int givenAngle) { //Edit Coefficent as 0 and -28 and run both tests , left orientation +90 , right orientation -90
  int koefL = -28; 
  int koefR = 28; 
  if ( 0.0 <= givenAngle < 90.0 ) {
    int totalAngle = givenAngle + 90 + koefL; // turn left
    servo.write(totalAngle);
    delay(3000);
    goBackward();
    delay(1000);
    goNothing();
  }
  else if (90.0 <= givenAngle < 180){
    int totalAngle = givenAngle - 90; // turn Right
    servo.write(totalAngle);
    delay(3000);
    goForward();
    delay(1000);
    goNothing();
    }
  else if(180.0 <= givenAngle < 270.0) {
    int totalAngle = givenAngle - 90 + koefL; // turn left
    servo.write(totalAngle);
    delay(3000);
    goForward();
    delay(1000);
    goNothing();
  }
  else if( 270.0 <= givenAngle < 360.0 ){
    int totalAngle = givenAngle - 270 + koefL; // turn left
    servo.write(totalAngle);
    delay(3000);
    goBackward();
    delay(1000);
    goNothing();
  }
  else {
  }
  
}

void Stop() //stop the motor
{
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(EnA, HIGH);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    int k = servo.read(); //Gets the Servo Motor angle
    
    //Check whether 1 cycle turn is completed. SCAN WILL BE COMPLETED HERE
    if(angle < OldAngle && angle > 0.0){
       //Serial.println("\n\n\nSample number is : ");
       //Serial.print(i);
       
       //perform data processing here...
       
       //
       // Find max distance
       //---------------------------------------------------------
       float max_v = 0;
       int max_i = 0;
       for ( int j = 0; j <i ; j++ )   //sizeof(arr) / sizeof(arr[0])
       {
          if ( distArray[j] > max_v )
          {
          max_v = distArray[j];
          max_i = j;
          }
       }
       // Find max distance
       //---------------------------------------------------------
       
       int anglerequired = angleArray[max_i];
       float maxfindDist = max_v;
       turnSides(anglerequired);

       // Print out the detailed variables applied there
       Serial.print(maxfindDist);
       Serial.print("\t");
       Serial.print(anglerequired);
       Serial.print("\t");
       Serial.print(k);
       Serial.print("\t");
       Serial.print(i);
       i = 0;
       distArray[100] = { 0 }; //Set every value to zero again
       angleArray[100] = { 0 }; //Set every value to zero again  FOR MEMORY EFFICENY
    }
    //Serial.println("\nAngle : ");
    //Serial.print(angle);
    
    // Save the old value as Old Angle for the further comparison
    OldAngle = angle;
    //Store the data in the array
    angleArray[i] = angle;
    distArray[i] = distance;
    i = i+1;
    
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}
