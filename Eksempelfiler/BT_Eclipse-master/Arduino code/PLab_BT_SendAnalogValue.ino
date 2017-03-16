/*
 */

#include <SoftwareSerial.h>
#include <PLabBTSerial.h>

const int txPin = 2; // Connected to tx on bt unit
const int rxPin = 3; // Connected to rx on bt unit
const int potmeterPin = A0;     // the Potmeter pin

int previousSensorValue = 0;

PLabBTSerial btSerial(txPin, rxPin);

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
    int sensorValue = analogRead(potmeterPin);
    if (sensorValue != previousSensorValue) {
      BTSerialSendMessage("#analogValueChanged",sensorValue);
      previousSensorValue = sensorValue;
      delay(100);
    }
}

