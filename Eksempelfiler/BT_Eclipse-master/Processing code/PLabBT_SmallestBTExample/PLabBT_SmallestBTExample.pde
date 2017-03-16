/*
 * The following example shows how to communicate with a bluetooth device.
 * The code is written so it is easy to run on both a computer and through the
 * PLab app. 
 *
 * When running in app, ONLY this file is neccesary to run.
 * AppReplacer is only used to run on computer.
 */
//-------- Always include the code from here -----------
private PLabBridge pBridge = null;;

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
// -------------------- Until here --------------------
// Your code starts here.
// Make sure to implement method messageReceived(String,int).
// Also include a call to initializeSerial() in setup().

public void messageReceived(String message, int value) {
  println("Message:",message,",",value); 
}

void setup() {
  // Setup a size for the sketch
  size(500,200);
  // Call setupSerial to run serial communication with com port. Comment out/remove when moving code to mobile device
  initializeSerial();  // Must always be included...
}

void draw() {
  fill(128); background(0); textSize(16);
  text("Press to send test message.",25,100);
  stroke(255);
  fill(255);
  rect(25,25,50,50);
}


void mouseClicked() {
  // Send a message to bt unit
  if (pBridge != null) {
    sendMessage("#Hello",42);
  }
}