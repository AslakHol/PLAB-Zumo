// This is the minimum amount of code for accepting input from the bluetooth connected app
// Ok it also includes things like serial logging

#include <SoftwareSerial.h>
#include <PLabBTSerial.h>

// Pins for bluetooth
const int BT_RX = 0;
const int BT_TX = 1;

PLabBTSerial btSerial (BT_TX, BT_RX);

void setup() {
  Serial.begin (9600);
  btSerial.begin (9600);
}

void loop() {
  int availableCount = btSerial.available();
  if (availableCount > 0) {
    char text[availableCount];
    
    btSerial.read(text, availableCount);
    
    readCommand(text);
  }  
}

void readCommand (char *text) {
  Serial.print ("Received: ");
  Serial.println (text);
  if (0 == strcmp ("SENTRY", text)) {
    engageSentry();
  } else if (0 == strcmp("SEEKER", text)) {
    engageSeeker();
  } else {
    Serial.print ("Unknown command");
  }
}

void engageSentry() {
  // Change parameters to engage sentry mode
}

void engageSeeker() {
  // This is the default mode in the app
  // Change parameters to engage seeker mode
}


