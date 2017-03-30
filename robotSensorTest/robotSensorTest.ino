/*
 * Demo line-following code for the Pololu Zumo Robot
 *
 * This code will follow a black line on a white background, using a
 * PID-based algorithm.  It works decently on courses with smooth, 6"
 * radius curves and has been tested with Zumos using 30:1 HP and
 * 75:1 HP motors.  Modifications might be required for it to work
 * well on different courses or with different motors.
 *
 * http://www.pololu.com/catalog/product/2506
 * http://www.pololu.com
 * http://forum.pololu.com
 */

#include <NewServo.h>
#include <NewPing.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>





/*
Drive forward and turn left or right when border is detected
  - Only reflectionsensor 0 and 5 are used.
*/
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
 
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1800 // 
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     200 // ms

 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
 
ZumoReflectanceSensorArray sensors;





//ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 400;
// Set constants for PID control
const float KP = 0.5;  // Proportional constant
const float KD = 4;    // Derivative constant
const int SV = 2500; // Set-value for position (in the middle of sensors)

const int MAX_DISTANCE = 70;
const int echoPinL = 3;
const int triggerPinL = 4;
const int echoPinR = 5;
const int triggerPinR = 6;
NewPing sonarR(triggerPinR, echoPinR, MAX_DISTANCE);
NewPing sonarL(triggerPinL, echoPinL, MAX_DISTANCE);

const int servoPin = 100;
int servoStepDeg = 7;
NewServo servo;
int servoAngle = 0;


int lastError = 0;

//Angle-PID
const int motorConstForConstantFacing = 13;
const int angleKP = 5;
int prevSpeed = 0;
const int angleKD = 0;

float lastSeenDir = 1;

void setup()
{
  Serial.begin(9600);
  pinMode(echoPinL, INPUT);
  pinMode(triggerPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(triggerPinR, OUTPUT);
  /*pinMode(servoPin, OUTPUT);
  servo.attach(servoPin);
  servo.write(20);*/
  button.waitForButton();


  sensors.init();
}

void alignForPotentialMurder(int& LSpeed, int& RSpeed, float distance){
   int angleError = 60-servoAngle;
   LSpeed = angleKP*(angleError);
   RSpeed = angleKP*(-angleError);
   servoAngle += angleError*0.2;
   servo.write(servoAngle);
   if(angleError < 2){
    servo.write(60);
   }
}

void sweep(int& LSpeed, int& RSpeed){
  servoAngle += servoStepDeg;
  if(servoAngle > 120){
    servoAngle = 120;
    servoStepDeg *= -1;
  }else if(servoAngle < 0){
    servoAngle = 0;
    servoStepDeg *= -1;
  }
  servo.write(servoAngle);
}

void kill(int& LSpeed, int& RSpeed){
  LSpeed = MAX_SPEED;
  RSpeed = MAX_SPEED;
}

void spin(int& LSpeed, int& RSpeed){
  LSpeed = -150;
  RSpeed = 150;
}

void search(int& LSpeed, int& RSpeed){
  int searchSpeed = MAX_SPEED;
  float distanceL = sonarL.ping_cm();
  float distanceR = sonarR.ping_cm();
  if(!distanceR && !distanceL){
    LSpeed = -searchSpeed*lastSeenDir;
    RSpeed = searchSpeed*lastSeenDir;
  }else if (distanceR && distanceL){
    int constant = 50;
    float error = distanceL - distanceR;
    LSpeed = searchSpeed+(error*constant);
    RSpeed = searchSpeed-(error*constant);
    lastSeenDir = (distanceL - distanceR)/abs(distanceL - distanceR);
  }else{
    LSpeed = searchSpeed-((distanceL - distanceR)*0.5);
    RSpeed = searchSpeed+((distanceL - distanceR)*0.5);
    lastSeenDir = (distanceL - distanceR)/abs(distanceL - distanceR);
  }
  Serial.println("Last seen: "+String(lastSeenDir)+"sensor"+String((distanceL - distanceR)));
}

void destroy(int& LSpeed, int& RSpeed){
  
}

void loop()
{
  int LSpeed = 0;
  int RSpeed = 0;

  float distanceR = sonarR.ping_cm();
  float distanceL = sonarL.ping_cm();
  //Serial.println(String(distanceL)+"\t"+String(distanceR));

  //search(LSpeed,RSpeed);
  //motors.setSpeeds(LSpeed,RSpeed);

  sensors.read(sensor_values);

  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }

  

  
  delay(40);
}
