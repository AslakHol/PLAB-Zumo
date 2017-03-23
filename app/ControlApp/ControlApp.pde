private PLabBridge pBridge;

void bindPLabBridge (PLabBridge bridge) {
  // Bind the bridge to the instance
  pBridge = bridge;
  // Set the size based on window size
  size (bridge.getWidth (), bridge.getHeight ());
  btnChanged ();
}

// To get relative positions, we define world size
int worldWidth = 100;
int worldHeight = 100;

// We define properties of our button in the world system
int buttonX = 25;                         // worldWidth / 4
int buttonY = 25;                         // worldHeight / 4
int buttonSize = 50;                      // worldWidth / 2 or worldHeight / 2
boolean buttonPressed = false;
boolean isSentryMode = false; 

// Conversion from world coordinates to screen coordinates
int bToScreenX (int x) {
  // Explicit cast done because of app javascript behaviour
  return (int) (width * x / worldWidth);
}
int bToScreenY (int y) {
  // Explicit cast done because of app javascript behaviour
  return (int) (height * y / worldHeight);
}

// Conversion from screen coordinates to world coordinates
int bToWorldX (int sx) {
  // Explicit cast done because of app javascript behaviour
  return (int) (worldWidth * sx / width);
}
int bToWorldY (int sy) {
  // Explicit cast done because of app javascript behaviour
  return (int) (worldHeight * sy / height);
}

// setup () is only used for testing purposes
void setup () {
  setupSerial();
  // This size set will be overridden when bridge is bound
  size (240, 360);
}

// Does the drawing. Functions like loop in Arduino code
void draw () {
  // Run update () to check if button is pressed
  bUpdate ();
  // grey background
  background (#7F7F7F);
  // black stroke around the button
  stroke (#000000);
  if (buttonPressed) {
    // Pressed -> black button
    fill (#000000);
  } else {
    if (isSentryMode) {
      fill (#00FF00); // Sentry mode == green button
    } else {
      fill(#FF0000); // Sentry mode == red button
    }
  }
  // Draw the button
  rect (bToScreenX (buttonX), bToScreenY (buttonY), bToScreenX (buttonSize), bToScreenY (buttonSize));
}

void bUpdate () {
  boolean oldPressed = buttonPressed;
  // Default press status to false
  buttonPressed = false;
  // If mouse is pressed
  if (mousePressed) {
    buttonPressed = mouseInBox();
    if (oldPressed != buttonPressed) {
      buttonPressed();
    }
  }  
}

void toggleIsSentryMode() {
  isSentryMode = !isSentryMode;
}

boolean mouseInBox() {
  // Translate mouse coordinates (screen coordinates) to world coordinates
    int mX = bToWorldX (mouseX);
    int mY = bToWorldY (mouseY);
    // Check if mouse is within bounds in x direction
    buttonPressed = mX >= buttonX && mX <= (buttonX + buttonSize);
    // Check if mouse is within bounds in y direction
    buttonPressed = buttonPressed && mY >= buttonY && mY <= (buttonY + buttonSize); 
    return buttonPressed; 
}

void buttonPressed() {
  isSentryMode = !isSentryMode;
  btnChanged();
}

void btnChanged () {
  // This is called when status of button has changed. Notify bridge
  // ALLWAYS CHECK FOR NULL TO AVOID ERRORS

  if (pBridge != null) {
    String send;
    if (isSentryMode) {
      send = "SENTRY";
    } else {
      send = "SEEKER";
    }
    pBridge.send (send);
  }
}