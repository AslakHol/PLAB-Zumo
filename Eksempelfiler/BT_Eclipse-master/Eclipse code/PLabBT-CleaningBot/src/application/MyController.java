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
	private PLabSerial serialConnection = new PLabSerial();
	private ProgressBar[] radarPlotBars= new ProgressBar[7];
	// private int[] radarPlotValues= {0,30,60,90,120,150,180};	
	int latestDistance = 0;
	@FXML
    private TextField serialPortNameField;
	
	
	@FXML
    private ProgressBar bar0;
    
	@FXML
    private ProgressBar bar30;
	
	@FXML
    private ProgressBar bar60;
    
	@FXML
    private ProgressBar bar90;
	
	@FXML
    private ProgressBar bar120;
	
	@FXML
    private ProgressBar bar150;
    
	@FXML
    private ProgressBar bar180;
	
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
		radarPlotBars[0] = bar0;
		radarPlotBars[1] = bar30;	
		radarPlotBars[2] = bar60;
		radarPlotBars[3] = bar90;	
		radarPlotBars[4] = bar120;	
		radarPlotBars[5] = bar150;
		radarPlotBars[6] = bar180;
		initRadarBars();
	}
	
	void initRadarBars() {
	  for (int i=0;i<7;i++) {
		radarPlotBars[i].setProgress(0.0);
	  }
	}
	public void messageReceived(String message, int value) {
		if (message.equals("#distance")) {
			latestDistance = value; return;
		}
		if (message.equals("#angle")) {
			initRadarBars();
			plotRadar(value, latestDistance); return;
		}
	}

	
	@FXML
	void speed0() {
	    serialConnection.sendMessage("#speed",0);
	}
	
	@FXML
	void speed100() {
	    serialConnection.sendMessage("#speed",100);
	}
	
	@FXML
	void speed200() {
	    serialConnection.sendMessage("#speed",200);
	}
	
	@FXML
	void speed300() {
	    serialConnection.sendMessage("#speed",300);
	}
	
	@FXML
	void speed400() {
	    serialConnection.sendMessage("#speed",400);
	}
	
	void plotRadar(int angle, int distance) {
		int angle30 = angle / 30;
		if (angle30 > 6) 
		    {angle30 = 6;};
		double value = distance / 100.0;
		radarPlotBars[angle30].setProgress(value);
	}
}
