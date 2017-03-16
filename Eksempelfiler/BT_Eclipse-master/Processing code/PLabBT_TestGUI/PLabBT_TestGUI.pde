//
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
String labelString1 = "Drag slider to change bar.";

int[] sliderData = {10,240,250,30,50};  // The last number is initial slider value 0-100.

int[] barFrame = {10,200,250,30};
 
void setup() {
  size(300,400);              // Canvas size is 200 x 200 pixels.
  background(192);            // Background color is lightgrey (192).
  stroke(0);                  // Stroke color is black (0)
}

void draw() {    // Called for every frame
  drawText(textFrame1,textString1);
  drawButton(redButtonFrame,redButtonString);
  drawButton(greenButtonFrame,greenButtonString);
  drawRectangle(redLedFrame,redLedColor);
  drawRectangle(greenLedFrame,greenLedColor);  
  drawLabel(labelFrame1,labelString1);  
  drawSlider(sliderData);
  int sliderValue = sliderData[4];
  drawBar(barFrame,sliderValue);
}

void mousePressed() {
   if (mouseInside(redButtonFrame)) {
     if (redLedColor == red) {
        textString1 = "Red LED off";
        redLedColor = black; 
     } else {
        textString1 = "Red LED on";
        redLedColor = red;  
     };
   } else 
   if (mouseInside(greenButtonFrame)) {
     if (greenLedColor == green) {
        textString1 = "Green LED off";
        greenLedColor = black; 
     } else {
        textString1 = "Green LED on";
        greenLedColor = green;  
     };  
   }   
}

void mouseReleased() {
}