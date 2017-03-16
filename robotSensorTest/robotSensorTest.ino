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

ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 300;
// Set constants for PID control
const float KP = 0.5;  // Proportional constant
const float KD = 4;    // Derivative constant
const int SV = 2500; // Set-value for position (in the middle of sensors)

const int MAX_DISTANCE = 100;
const int triggerPin = 6;
const int echoPin = 3;
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

const int servoPin = 2;
int servoStepDeg = 10;
NewServo servo;
int servoAngle = 0;

int lastError = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin);
  servo.write(60);
  button.waitForButton();
}

void sweep(int& LSpeed, int& RSpeed){
  servoAngle += servoStepDeg;
  if(servoAngle > 180){
    servoAngle = 180;
    servoStepDeg *= -1;
  }else if(servoAngle < 0){
    servoAngle = 0;
    servoStepDeg *= -1;
  }
  servo.write(servoAngle);
  if(servoAngle > 60){
    RSpeed = 200;
    LSpeed = -200;
  }else{
    RSpeed = -200;
    LSpeed = 200;
  }
}

void kill(int& LSpeed, int& RSpeed){
   int angleError = 60-servoAngle;
   LSpeed = -100*angleError;
   RSpeed = -100*angleError;
   servoAngle += angleError*0.2;
   servo.write(servoAngle);
}

void loop()
{
  int LSpeed = 100;
  int RSpeed = 100;
  
  float distance = sonar.ping_cm();
  Serial.println(distance);
  //sweep(LSpeed, RSpeed);
  
  if (distance){
    kill(LSpeed, RSpeed);
  }else{
    sweep(LSpeed, RSpeed);
  }
  
  motors.setSpeeds(LSpeed, RSpeed);
  delay(50);
}
