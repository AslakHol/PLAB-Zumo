package application;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.Slider;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

public class MyController {
	
	private PLabSerial serialConnection = new PLabSerial();;
	private int previousSliderValue = 0;
	@FXML
    private TextField serialPortNameField;
	
	@FXML
    private Slider mySlider;
	
	@FXML
	void initialize() {
		String portName;
		if(serialConnection.openPLabPort()) {	
		    portName = serialConnection.getOpenPortName();}
		else {
			portName = "No PLab BT device found.";
		};
		serialPortNameField.setText(portName);	
		
		mySlider.valueProperty().addListener(new ChangeListener<Object>() {
            @Override
            public void changed(ObservableValue<?> arg0, Object arg1, Object arg2) {
                sliderValueChanged();
            }
        });
		
		
	}
	
	@FXML
	void sliderValueChanged() {
		int analogValue = (int) mySlider.getValue();
		if (previousSliderValue != analogValue) {
		    serialConnection.sendMessage("#analogValueChanged",analogValue);
		    previousSliderValue = analogValue;
		}

	}
	
	
}
