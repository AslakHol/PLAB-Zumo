/*/
MCP23008 i2c pin expander: 
 Connect pin #1 of the expander to Analog 5 (i2c clock)
 Connect pin #2 of the expander to Analog 4 (i2c data)
 Connect pins #3, 4 and 5 of the expander to ground (address selection)
 Connect pin #6 and 18 of the expander to 5V (power and reset disable)
 Connect pin #9 of the expander to ground (common ground)
 Output #0 is on pin 10 so connect a LED here via a 220 ohm resistor to ground.
 */
#include <NewPing.h>
#include <NewServo.h>
#include <Pushbutton.h>
#include <PLab_ZumoMotors.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <SoftwareSerial.h>
#include <PLabBTSerial.h>

#include <Wire.h>
#include "Adafruit_MCP23008.h"

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

const int txPin = 2; // Connected to tx on bt unit
const int rxPin = 3; // Connected to rx on bt unit

PLabBTSerial btSerial(txPin, rxPin);

NewPing sonar(triggerPin, echoPin, maxDistance);
NewServo myServo; 

Pushbutton button(ZUMO_BUTTON); 

int degreesServo = 0;
int degreesStep = 10;

PLab_ZumoMotors plab_Motors;

Adafruit_MCP23008 mcp;
//...........................................................................
// Always include this method.
// It reads from the BT port and calls BTSerialMessageReceived.
// 
char msg[100];
void updateBTSerial() {
  int availableCount = btSerial.available();
  if (availableCount > 0) {
    btSerial.read(msg, availableCount);
    char *divided = strchr(msg,',');
    int msgValue = 0;
    if (divided != 0) {
       divided[0] = 0; divided++;
       String str(divided);
       msgValue = str.toInt();
    };
    String msgString(msg);
    BTSerialMessageReceived(msgString,msgValue);   
  }
}
//...........................................................................

//...........................................................................
// Always include these two methods .
// They send a message to the BT port, without or with an int value
// 
void BTSerialSendMessage(String msgString) {
  btSerial.println(msgString); 
}

void BTSerialSendMessage(String msgString,int msgValue) {
  btSerial.print(msgString); 
  btSerial.print(",");
  btSerial.println(msgValue);
}
//...........................................................................


void setup() {
  sensors.init(QTR_NO_EMITTER_PIN);  // 
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  myServo.attach(servoPin); 
  myServo.write(90);
  button.waitForButton(); // start when button pressed
  btSerial.begin(9600); // Open serial communication to Bluetooth unit
  mcp.begin();      // use default address 0
  mcp.pinMode(0, OUTPUT);
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
    // digitalWrite(ledPin,LOW); 
    mcp.digitalWrite(0,LOW); 
   } else {
    // digitalWrite(ledPin,HIGH);
    mcp.digitalWrite(0,HIGH); 
   }
   return distance;
}

void BTSerialMessageReceived(String msgString,int msgValue) {
  Serial.print("Message:"); Serial.print(msgString); // Debug print
  Serial.print(", Value:"); Serial.println(msgValue);  // Debug print
  if (msgString == "#speed") {
    REVERSE_SPEED  = msgValue; // 0 is stopped, 400 is full speed
    TURN_SPEED     = msgValue;
    FORWARD_SPEED  = msgValue; return;
  } 
}

void loop() {
   updateBTSerial();  // Check if we have input on the BT serial port.
   stepServo(); 
   int distance = sonarDistance(); 
   if (distance > 0) {
      int actual_degrees_servo = degreesServo + SERVO_OFFSET;
      if (actual_degrees_servo > 90) {
         plab_Motors.turnLeft(TURN_SPEED,actual_degrees_servo-90);
         BTSerialSendMessage("#distance",distance);       
         BTSerialSendMessage("#angle",actual_degrees_servo);
         degreesServo = 90 - SERVO_OFFSET;
      } else if (actual_degrees_servo < 90) {
         plab_Motors.turnRight(TURN_SPEED,90-actual_degrees_servo);
         BTSerialSendMessage("#distance",distance);       
         BTSerialSendMessage("#angle",actual_degrees_servo);
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


