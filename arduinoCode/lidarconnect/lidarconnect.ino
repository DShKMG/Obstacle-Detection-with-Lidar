
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>
#include <Servo.h>

// You need to create an driver instance 
RPLidar lidar;
Servo serv;

#define RPLIDAR_MOTOR 12 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal 
float OldAngle = 0;
int i =0;
float AngleArr[100];
                        
void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial.begin(9600);
  lidar.begin(Serial2);

  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  serv.attach(11);
  
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    
    //Check whether 1 cycle turn is completed
    if(angle < OldAngle && angle > 0.0){
       Serial.println("\n\n\nSample number is : ");
       Serial.print(i);
       i = 0;
       
       /*serv.write(90);
       delay(3000);
       serv.write(135);
       delay(3000);
       serv.write(45);
       delay(3000);*/
       //perform data processing here...
    }
    //Serial.println("\nAngle : ");
    //Serial.print(angle);
    
    // Save the old value as Old Angle for the further comparison
    OldAngle = angle;
    //Store the data in the array
    AngleArr[i] = angle;
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
