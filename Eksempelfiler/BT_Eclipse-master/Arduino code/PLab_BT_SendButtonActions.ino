/*
 */

#include <SoftwareSerial.h>
#include <PLabBTSerial.h>
#include "PLab_PushButton.h"

const int txPin = 2;  // Tx pin on Bluetooth unit
const int rxPin = 3;  // Rx pin on Bluetooth unit

const int redButtonPin = 6;     // the number of the pushbutton pin
const int greenButtonPin = 7;   // the number of the pushbutton pin

PLabBTSerial btSerial(txPin, rxPin);
PLab_PushButton redButton(redButtonPin);     // Create a PushButton object.
PLab_PushButton greenButton(greenButtonPin); // Create a PushButton object.

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

void setup()
{
    btSerial.begin(9600); // Open serial communication to Bluetooth unit
}

void loop() {
  redButton.update();    // Update: Read the red button switch.
  greenButton.update();  // Update: Read the green button switch.

  if (redButton.pressed()) {  // Was the red button pressed?  
     BTSerialSendMessage("#redButtonPressed");
  };   
  if (greenButton.pressed()) {  // Was the green button pressed?  
     BTSerialSendMessage("#greenButtonPressed");
  };  
}

