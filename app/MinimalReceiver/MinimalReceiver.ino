// This is the minimal code needed to receive signals from the bluetooth arduino

const int IN_PIN = 8;
const int OUT_PIN = 7; 

boolean isHIGH = false; 

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  pinMode(IN_PIN, INPUT);
  digitalWrite(OUT_PIN, LOW); 
}

void loop() {
  boolean prevIsHIGH = isHIGH; 
  
  if (digitalRead(IN_PIN) == HIGH) {
    isHIGH = true; 
  } else {
    isHIGH = false; 
  }
  if (prevIsHIGH != isHIGH) {
    instructionChange();
  }
}

void instructionChange() {
  digitalWrite(OUT_PIN, !digitalRead(OUT_PIN));
}

