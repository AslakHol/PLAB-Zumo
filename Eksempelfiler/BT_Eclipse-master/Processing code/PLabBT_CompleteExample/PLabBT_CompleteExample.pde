/*
 * The following example shows how to communicate with a bluetooth device.
 * The code is written so it is easy to run on both a computer and through the
 * PLab app. 
 *
 * When running in app, ONLY this file is neccesary to run.
 * AppReplacer is only used to run on computer.
 */
//-------- Always include the code from here -----------
private PLabBridge pBridge;

void bindPLabBridge (PLabBridge bridge) {
  pBridge = bridge;
  
  // Subscribe to messages. Print incomming messages and change color of drawing
  bridge.subscribeMessages(new PLabRecv() {
    public void receive (String string) {
      String[] q = splitTokens(trim(string),",");
      if(q.length > 0) {
        int value = 0;
        if (q.length > 1) {
          value = parseInt(q[1],0);
        }
        messageReceived(q[0],value);
      }
    }
  });
  // Not striclty needed, but included as it is needed to fill screen on mobile device
  size(bridge.getWidth (), bridge.getHeight ());
}

void sendMessage(String message, int value) {
  value = int(value);
  String m = message + "," + value;
  pBridge.send(m);
}

void initializeSerial() {
  final Class<?>[] parameterType = {};
  try
    {this.getClass().getMethod("setupSerial", parameterType);}
  catch (Exception e)
    {return;};
  setupSerial();
}
// ----------------------- Until here ----------------------
// Your code starts here.
// Make sure to implement method messageReceived(String,int).
// Also include a call to initializeSerial() in setup().//

// ----------------------------------------------------------
//  Minimal Processing GUI library.
//  PLab 2017
//
//  Dag Svanæs, IDI, NTNU.
// 

int[] white = {255,255,255};
int[] black = {0,0,0};
int[] red = {255,0,0};
int[] green = {0,255,0};
int[] blue = {0,0,255};
int[] yellow = {255,255,0};
int[] gray = {128,128,128};
int[] lightGray = {192,192,192};

boolean inside(int x,int y, int x0,int y0,int w,int h) {
  return (((x >= x0) && (x < (x0+w))) && 
      ((y >= y0) && (y < (y0+h))));
}

boolean mouseInside(int[] rect) {
  return inside(mouseX, mouseY, rect[0],rect[1],rect[2],rect[3]);
}

void drawButton(int[] xywh, String buttonText) {
    boolean mousePressedInButton = mouseInside(xywh) && mousePressed;
    stroke(0);
    if (mousePressedInButton) {
      fill(0); }
    else {
      fill(255); 
    } 
    rect(xywh[0],xywh[1],xywh[2],xywh[3],10);
    if (mousePressedInButton) {
      fill(255); }
    else {  
      if (mouseInside(xywh)) {
      fill(128);
        } else {
      fill(0); 
      ;}
    };
    int tSize = xywh[3] * 3 / 5;
    textSize(tSize);
    textAlign(CENTER, CENTER);
    text(buttonText,xywh[0]+xywh[2]/2,xywh[1]+xywh[3]/2-2);
}

void drawLabel(int[] xywh, String labelText) {
    stroke(0);
    fill(0);  
    int tSize = xywh[3] * 3 / 5;
    textSize(tSize);
    textAlign(LEFT, CENTER);
    text(labelText,xywh[0]+2,xywh[1]+xywh[3]/2-2);
}

void drawText(int[] xywh, String buttonText) {
    stroke(0);
    fill(255);  
    rect(xywh[0],xywh[1],xywh[2],xywh[3]);
    drawLabel(xywh, buttonText);
}
  
void drawRectangle(int[] xywh, int[] drawColor) {
    stroke(0);
    fill(drawColor[0],drawColor[1],drawColor[2]);  
    rect(xywh[0],xywh[1],xywh[2],xywh[3]);
}

void drawSlider(int[] xywhp) {
    stroke(0);
    drawRectangle(xywhp,white);
    int[] square = new int[4];
    square[1] = xywhp[1] + 1;
    square[2] = xywhp[3] - 2;
    square[3] = square[2];
    boolean mousePressedInSlider = mouseInside(xywhp) && mousePressed;
    if (mousePressedInSlider) {
       xywhp[4] = 100*(mouseX - xywhp[0] - (xywhp[3]/2))/(xywhp[2]-xywhp[3]-2);
       if(xywhp[4] < 0) {
         xywhp[4] = 0;
       };
       if(xywhp[4] > 100) {
         xywhp[4] = 100;
       };      
    };
    square[0] = (xywhp[0] + 1) + (xywhp[2]-xywhp[3])*xywhp[4]/100;
    drawRectangle(square,black);
}

void drawBar(int[] xywh, int value) {
    stroke(0);
    drawRectangle(xywh,white);
    int barWidth = xywh[2];
    xywh[2] = barWidth * value / 100;
    drawRectangle(xywh,black);
    xywh[2] = barWidth;
}
// End of GUI library
//------------------------------------------------------------------------

int[] textFrame1 = {10,15,250,30};
String textString1 = "...";

int[] redButtonFrame = {25,130,100,25};
String redButtonString = "Red LED";

int[] greenButtonFrame = {145,130,100,25};
String greenButtonString = "Green LED";

int[] redLedFrame = {50,60,50,50};
int[] redLedColor = black;   // RGB white..

int[] greenLedFrame = {170,60,50,50};
int[] greenLedColor = black;   // RGB white..

int[] labelFrame1 = {10,165,100,30};
String labelString1 = "Drag slider to control servo.";

int[] sliderData = {10,240,250,30,50};  // The last number is initial slider value 0-100.
int previousSliderValue = 0;

int[] barFrame = {10,200,250,30};
 
int barValue = 0;

void setup() {
  initializeSerial();         // Must always be included...
  size(300,400);              // Canvas size is 300 x 400 pixels.
  background(192);            // Background color is lightgrey (192).
  stroke(0);                  // Stroke color is black (0)
  if (pBridge == null) {
    textString1 = "No connection.";
  } else {
    textString1 = "Bluetooth connected.";
  }
}

void draw() {    // Called for every frame
  drawText(textFrame1,textString1);
  drawButton(redButtonFrame,redButtonString);
  drawButton(greenButtonFrame,greenButtonString);
  drawRectangle(redLedFrame,redLedColor);
  drawRectangle(greenLedFrame,greenLedColor);  
  drawLabel(labelFrame1,labelString1);  
  drawSlider(sliderData);
  drawBar(barFrame,barValue);
  
  checkForSliderChanges();
}

void checkForSliderChanges() {
    int sliderValue = sliderData[4];
    if (sliderValue != previousSliderValue) {
      int analogValue = 180 * sliderValue / 100;
      sendMessage("#analogValueChanged",analogValue);
      previousSliderValue = sliderValue;
    }    
}

public void messageReceived(String message, int value) {
    if (message.equals("#redButtonPressed")) {
      toggleRedLed(); return;
    };
    if (message.equals("#greenButtonPressed")) {
      toggleGreenLed(); return;
    };
    if (message.equals("#analogValueChanged")) {
      barValue = value * 100 / 1024; return;
    };
}

void mousePressed() {
   if (mouseInside(redButtonFrame)) {
     sendMessage("#redButtonPressed",0);
   } else 
   if (mouseInside(greenButtonFrame)) { 
     sendMessage("#greenButtonPressed",0);
   }   
}

void toggleRedLed() {
     if (redLedColor == red) {
        redLedColor = black;  // Red..
     } else {
        redLedColor = red;  // Red..
     };
}

void toggleGreenLed() {
     if (greenLedColor == green) {
        greenLedColor = black;  // Green..
     } else {
        greenLedColor = green;  // Green...
     };
}

void mouseReleased() {
}