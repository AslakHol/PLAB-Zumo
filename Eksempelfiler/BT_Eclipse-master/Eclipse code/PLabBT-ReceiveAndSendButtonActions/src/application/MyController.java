package application;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;

public class MyController {
	
	private PLabSerial serialConnection = new PLabSerial();
	
	@FXML
    private Label redLedLabel;
	
	@FXML
    private Label greenLedLabel;
	
	@FXML
    private TextField serialPortNameField;
	
	@FXML
	void initialize() {
		String portName;
		if(serialConnection.openPLabPort()) {	
		    portName = serialConnection.getOpenPortName();}
		else {
			portName = "No PLab BT device found.";
		};
		serialPortNameField.setText(portName);
		serialConnection.addListener(this, "messageReceived");		
	}
	
	public void messageReceived(String message, int value) {
		if (message.equals("#redButtonPressed")) {
			arduinoRedButtonPressed(); return;
		}
		if (message.equals("#greenButtonPressed")) {
			arduinoGreenButtonPressed(); return;
		}
	}
	
	void arduinoRedButtonPressed() {
		if(redLedLabel.getTextFill() == Color.RED) {
			redLedLabel.setTextFill(Color.BLACK);}
		else {
			redLedLabel.setTextFill(Color.RED);
		}
	}
	
	void arduinoGreenButtonPressed() {
		if(greenLedLabel.getTextFill() == Color.GREEN) {
			greenLedLabel.setTextFill(Color.BLACK);}
		else {
			greenLedLabel.setTextFill(Color.GREEN);
		}
	}
	
	@FXML
	void redButtonPressed() {
	    serialConnection.sendMessage("#redButtonPressed",0);
	}
	
	@FXML
	void greenButtonPressed() {
	    serialConnection.sendMessage("#greenButtonPressed",0);
	}
	
}
