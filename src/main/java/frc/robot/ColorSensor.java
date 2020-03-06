package frc.robot;

// imports

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

public class ColorSensor{
  private CANSparkMax wofMotor = new CANSparkMax(2, MotorType.kBrushless);
boolean runSensor=false;
// class variables
I2C color = new I2C(Port.kOnboard,0x52);
private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
private final ColorMatch m_colorMatcher = new ColorMatch();
// color.write(0x00,192);
private final Color kBlueTarget = ColorMatch.makeColor(0.121, 0.430, 0.447);
private final Color kGreenTarget = ColorMatch.makeColor(0.165, 0.587, 0.249);
private final Color kRedTarget = ColorMatch.makeColor(0.417, 0.390, 0.171);
private final Color kYellowTarget = ColorMatch.makeColor(0.320, 0.563, 0.114);
ColorSensor(){
//constructor
m_colorMatcher.addColorMatch(kBlueTarget);
m_colorMatcher.addColorMatch(kGreenTarget);
m_colorMatcher.addColorMatch(kRedTarget);
m_colorMatcher.addColorMatch(kYellowTarget);
wofMotor.restoreFactoryDefaults();
wofMotor.setIdleMode(IdleMode.kBrake);

}

public String getColor(){
String color;
String colorString;
Color detectedColor = m_colorSensor.getColor();
ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    m_colorMatcher.setConfidenceThreshold(0.01);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
return colorString;

}

public String spinWheel(){
String initialColor,currentColor;
initialColor=getColor();
SmartDashboard.putString("Init Color = ",initialColor);
String previousColor = getColor();

int i, count=0;

while (count<4){
    currentColor=getColor();
if (currentColor.equals(initialColor) && !previousColor.equals(initialColor)) {
    count++;
    SmartDashboard.putNumber("Count = ",count);
}
   previousColor=currentColor;
}
return initialColor;

} 




public void wofIn(){
  wofMotor.set(0.5);
}

public void wofOut(){
  wofMotor.set(-0.5);
}

public void wofStop(){
  wofMotor.set(0);
}


public void getGameData(){
  String gameData; 
  gameData = DriverStation.getInstance().getGameSpecificMessage(); 
  DriverStation.getInstance().
  
}


}

