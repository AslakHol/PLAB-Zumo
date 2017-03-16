package application;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;

public class MyController {
	
	private PLabSerial serialConnection = new PLabSerial();;
	
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
		serialPortNameField.setText(portName);;		
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
