package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;

public class Sonar {
    private AnalogInput ai;
    private MedianFilter medFilter = new MedianFilter(10);
    private double conversion = 1;
    private double  medVoltage; 
    Sonar(int pin){
        ai = new AnalogInput(pin);   

        (new Thread() {
            public void run() {
                while(true){
                 medVoltage=medFilter.calculate(ai.getVoltage());
                }
            }
           }).start();
    }

    public double getDistance(){
        return medVoltage*conversion;        
    }
}