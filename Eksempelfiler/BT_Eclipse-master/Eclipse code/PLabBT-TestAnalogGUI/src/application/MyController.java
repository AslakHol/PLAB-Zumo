package application;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.Slider;
import javafx.scene.control.ProgressBar;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

public class MyController {
	
	@FXML
    private TextField myTextField;
	
	@FXML
    private Slider mySlider;
	
	@FXML
    private ProgressBar myProgressbar;
	
	@FXML
	void initialize() {
		sliderValueChanged();
		mySlider.valueProperty().addListener(new ChangeListener<Object>() {
	            @Override
	            public void changed(ObservableValue<?> arg0, Object arg1, Object arg2) {
	                sliderValueChanged();
	            }
	        });
	}
	
	void setProgressBarValue(double progressValue) {
		myProgressbar.setProgress(progressValue);
	}
	
	@FXML
	void sliderValueChanged() {
		int analogValue = (int) mySlider.getValue();
		myTextField.setText(String.valueOf(analogValue));
		double progressValue = mySlider.getValue() / 180.0;
		setProgressBarValue(progressValue);
	}
	
	
}
