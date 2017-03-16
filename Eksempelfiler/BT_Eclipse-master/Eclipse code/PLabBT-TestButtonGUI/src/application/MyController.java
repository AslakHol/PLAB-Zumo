package application;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;

public class MyController {
	
	@FXML
    private Label redLedLabel;
	
	@FXML
    private Label greenLedLabel;
	
	@FXML
    private TextField serialPortNameField;
	
	@FXML
	void initialize() {
		serialPortNameField.setText("Hello World!");;		
	}
	
	@FXML
	void redButtonPressed() {
		if(redLedLabel.getTextFill() == Color.RED) {
			redLedLabel.setTextFill(Color.BLACK);}
		else {
			redLedLabel.setTextFill(Color.RED);
		}
	}
	
	@FXML
	void greenButtonPressed() {
		if(greenLedLabel.getTextFill() == Color.GREEN) {
			greenLedLabel.setTextFill(Color.BLACK);}
		else {
			greenLedLabel.setTextFill(Color.GREEN);
		}
	}
	
}
