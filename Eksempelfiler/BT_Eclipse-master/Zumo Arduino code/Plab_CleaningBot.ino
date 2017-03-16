/*
 */
#include <NewPing.h>
#include <NewServo.h>
#include <Pushbutton.h>
#include <PLab_ZumoMotors.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1800 // 
  
// these might need to be tuned for different motor types
int REVERSE_SPEED  = 200; // 0 is stopped, 400 is full speed
int TURN_SPEED     = 200;
int FORWARD_SPEED  = 200;

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

#define SERVO_OFFSET 20  // DS: Degrees offset of servo... error in hardware...
 
ZumoReflectanceSensorArray sensors;


ZumoMotors motors;

const int ledPin=A4;

const int echoPin = A1;
const int triggerPin = A1;
const int maxDistance = 50;

const int servoPin = 6;

NewPing sonar(triggerPin, echoPin, maxDistance);
NewServo myServo; 

Pushbutton button(ZUMO_BUTTON); 

int degreesServo = 0;
int degreesStep = 10;

PLab_ZumoMotors plab_Motors;

void setup() {
  sensors.init(QTR_NO_EMITTER_PIN);  // 
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  myServo.attach(servoPin); 
  myServo.write(90);
  button.waitForButton(); // start when button pressed
}

void stepServo() {
   degreesServo = degreesServo + degreesStep;
   if (degreesServo > 180) {
       degreesStep = -degreesStep;
       degreesServo = 180;
   } else if (degreesServo < 0) {
       degreesStep = -degreesStep;
       degreesServo = 0;
   } 
   myServo.write(degreesServo);
}

float sonarDistance() {
  // Gjør ett ping, og beregn avstanden
  unsigned int time = sonar.ping();
  float distance = sonar.convert_cm(time);
  if (distance == 0.0) { // sonar gives zero when outside range
    // Turn off LED and just go forward
    digitalWrite(ledPin,LOW); 
   } else {
    digitalWrite(ledPin,HIGH);
   }
   return distance;
}


void loop() {
   stepServo(); 
   int distance = sonarDistance(); 
   if (distance > 0) {
      int actual_degrees_servo = degreesServo + SERVO_OFFSET;
      if (actual_degrees_servo > 90) {
         plab_Motors.turnLeft(TURN_SPEED,actual_degrees_servo-90);
         degreesServo = 90 - SERVO_OFFSET;
      } else if (actual_degrees_servo < 90) {
         plab_Motors.turnRight(TURN_SPEED,90-actual_degrees_servo);
         degreesServo = 90 - SERVO_OFFSET;
      };
      myServo.write(degreesServo);
    }
   sensors.read(sensor_values);  
   if (sensor_values[0] < QTR_THRESHOLD) {
     plab_Motors.backward(REVERSE_SPEED, 10);
     plab_Motors.turnRight(TURN_SPEED,90);
   } else if (sensor_values[5] < QTR_THRESHOLD) {
     plab_Motors.backward(REVERSE_SPEED, 10);
     plab_Motors.turnLeft(TURN_SPEED,90);
   }
   else
   {
    // otherwise, go straight
     motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  } 
}


