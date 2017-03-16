package application;
import javafx.fxml.FXML;

import javafx.scene.control.TextField;
import javafx.scene.control.Slider;
import javafx.scene.control.ProgressBar;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;


public class MyController {
	
	private PLabSerial serialConnection = new PLabSerial();
	
	@FXML
    private TextField serialPortNameField;
	
	@FXML
    private ProgressBar myProgressbar;
	
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
	
	void setProgressBarValue(double progressValue) {
		myProgressbar.setProgress(progressValue);
	}
	
	public void messageReceived(String message, int value) {
	//	serialPortNameField.setText(message + " ,"+ value);
		if (message.equals("#analogValueChanged")) {
			double progress = (double) value / 1024.0;
			setProgressBarValue(progress); return;
		}

	}
	
	
}
