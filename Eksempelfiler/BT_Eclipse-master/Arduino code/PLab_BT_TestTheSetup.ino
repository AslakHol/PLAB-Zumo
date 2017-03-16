#include "PLab_PushButton.h"
#include <Servo.h>    // Inkluderer Servo-iblioteket i koden

const int redLedPin =  4;       // the number of the red LED pin
const int greenLedPin =  5;     // the number of the green LED pin
const int redButtonPin = 6;     // the number of the pushbutton pin
const int greenButtonPin = 7;   // the number of the pushbutton pin
const int potmeterPin = A0;     // the Potmeter pin
const int servoPin = 8;         // the number of the Servo out pin

PLab_PushButton redButton(redButtonPin);     // Create a PushButton object.
PLab_PushButton greenButton(greenButtonPin); // Create a PushButton object.

boolean redLedState = false;
boolean greenLedState = false;

Servo myServo;       // Lager et servo-objekt

void setup() {
  // initialize the LED pin as an output:
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  myServo.attach(servoPin); // Knytter servo-objektet til rikitig pinne 
}

void loop() {
  redButton.update();    // Update: Read the red button switch.
  greenButton.update();  // Update: Read the green button switch.
  
  if (redButton.pressed()) {  // Was the red button pressed?  
     redLedState = !redLedState;   // Toggle LED state.
     digitalWrite(redLedPin, redLedState);
  };  
    
  if (greenButton.pressed()) {  // Was the green button pressed?  
     greenLedState = !greenLedState;  // Toggle LED state.
     digitalWrite(greenLedPin, greenLedState);
  };  
    
   int sensorValue = analogRead(potmeterPin);
   int angle  = map(sensorValue, 0, 1023, 0, 179);
   myServo.write(angle);
   delay(15);
}
