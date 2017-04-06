// This app will connect to the bluetooth and use the portexpander

#include <SoftwareSerial.h>
#include <PLabBTSerial.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>


const int addr = 0; 
Adafruit_MCP23008 mcp;

// Pins for bluetooth
const int BT_RX = 0;
const int BT_TX = 1;

boolean sentryMode = false; 
const int blinkInterval = 200; 
int blinkTime;
int timeNow; 

PLabBTSerial btSerial (BT_TX, BT_RX);

void setup() {
  btSerial.begin (9600);
  mcp.begin(addr);
  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(2, OUTPUT);
  mcp.pinMode(3, OUTPUT);
  mcp.pinMode(4, OUTPUT);
  mcp.pinMode(5, OUTPUT);
  mcp.pinMode(6, OUTPUT);
  mcp.pinMode(7, OUTPUT);
  blinkTime = millis();
  timeNow = millis();
  mcp.digitalWrite(0, HIGH);
  mcp.digitalWrite(1, HIGH);
  mcp.digitalWrite(2, LOW);
  mcp.digitalWrite(3, LOW);
  mcp.digitalWrite(4, LOW);
  mcp.digitalWrite(5, LOW);
  mcp.digitalWrite(6, HIGH);
  mcp.digitalWrite(7, HIGH);
}

void loop() {
  int availableCount = btSerial.available();
  if (availableCount > 0) {
    char text[availableCount];
    
    btSerial.read(text, availableCount);
    
    readCommand(text);
  }
  timeNow = millis();
  if (sentryMode && timeNow >= blinkTime + blinkInterval) {
    blinkTime = millis();
    if (mcp.digitalRead(0) == LOW) {
      mcp.digitalWrite(0, HIGH);
      mcp.digitalWrite(1, HIGH);
      mcp.digitalWrite(2, HIGH);
      mcp.digitalWrite(3, HIGH);
      mcp.digitalWrite(4, HIGH);
      mcp.digitalWrite(5, HIGH);
      mcp.digitalWrite(6, HIGH);
      mcp.digitalWrite(7, HIGH);
     } else {
      mcp.digitalWrite(0, LOW);
      mcp.digitalWrite(1, LOW);
      mcp.digitalWrite(2, LOW);
      mcp.digitalWrite(3, LOW);
      mcp.digitalWrite(4, LOW);
      mcp.digitalWrite(5, LOW);
      mcp.digitalWrite(6, LOW);
      mcp.digitalWrite(7, LOW);
    }
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
  mcp.digitalWrite(4, LOW);
  sentryMode = true; 
}

void engageSeeker() {
  // This is the default mode in the app
  // Change parameters to engage seeker mode
  mcp.digitalWrite(4, HIGH);
  sentryMode = false; 
}

void toggleHairLights() {
    mcp.digitalWrite(2, LOW);
    mcp.digitalWrite(3, !digitalRead(3));
    mcp.digitalWrite(4, !digitalRead(4));
    mcp.digitalWrite(5, !digitalRead(5));
    mcp.digitalWrite(6, !digitalRead(6));
    mcp.digitalWrite(6, !digitalRead(7)); 
}

