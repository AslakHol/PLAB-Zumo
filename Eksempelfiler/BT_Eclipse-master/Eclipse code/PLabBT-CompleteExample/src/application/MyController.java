package application;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;
import javafx.scene.control.Slider;
import javafx.scene.control.ProgressBar;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

public class MyController {
	private int previousSliderValue = 0;
	private PLabSerial serialConnection = new PLabSerial();
	
	@FXML
    private Label redLedLabel;
	
	@FXML
    private Label greenLedLabel;
	
	@FXML
    private TextField serialPortNameField;
	
	@FXML
    private Slider mySlider;
	
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
		mySlider.valueProperty().addListener(new ChangeListener<Object>() {
            @Override
            public void changed(ObservableValue<?> arg0, Object arg1, Object arg2) {
                sliderValueChanged();
            }
        });
	}
	
	public void messageReceived(String message, int value) {
		if (message.equals("#redButtonPressed")) {
			arduinoRedButtonPressed(); return;
		}
		if (message.equals("#greenButtonPressed")) {
			arduinoGreenButtonPressed(); return;
		}
		if (message.equals("#analogValueChanged")) {
			double progress = (double) value / 1024.0;
			setProgressBarValue(progress); return;
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
	
	@FXML
	void sliderValueChanged() {
		int analogValue = (int) mySlider.getValue();
		if (previousSliderValue != analogValue) {
		    serialConnection.sendMessage("#analogValueChanged",analogValue);
		    previousSliderValue = analogValue;
		}

	}
	void setProgressBarValue(double progressValue) {
		myProgressbar.setProgress(progressValue);
	}
}
