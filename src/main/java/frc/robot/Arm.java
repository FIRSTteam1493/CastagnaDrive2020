package frc.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 
public class Arm{
    private TalonFX armMotor = new TalonFX(6);
    private int topPos = 74000, intakePos=74000,scorePos=57000, floorPos = -22000;
    private CANSparkMax shooterMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax wofMotor = new CANSparkMax(2, MotorType.kBrushless);
    private Solenoid armsol1 = new Solenoid(3);
    private Solenoid armsol2 = new Solenoid(6);
    private AnalogInput lsfAnalogIn = new AnalogInput(0);
    private AnalogInput lsrAnalogIn = new AnalogInput(1);
    private boolean lsf=false, lsr=false;
    public boolean pidRunning=false;
    private double lsfVolt, lsrVolt;
    
    // On Position:   solState1 = true       solState2 = false
    // Off Position:   solState1 = false     solState2 = true
    // start with brake on
    private boolean sol1State = true;
    private boolean sol2State = false;

Arm(){

    armMotor.configFactoryDefault();
    armMotor.configNeutralDeadband(0.005);
//   armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector , LimitSwitchNormal.NormallyOpen);
//    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMotor.set(ControlMode.PercentOutput, 0);
    
// slot 0 for magic motion by button    
    armMotor.config_kP(0,.5);
    armMotor.config_kD(0,.2);
    armMotor.config_kF(0,.05);
//    armMotor.config_kF(0,0.056);
    armMotor.configClosedLoopPeakOutput(0, 0.7);
    
// slot 1 for manual control    
    armMotor.config_kP(1,.4);
    armMotor.config_kD(1,0.1);
    armMotor.config_kF(0,0.125);
    armMotor.configClosedLoopPeakOutput(1,0.7);

//    armMotor.configClearPositionOnLimitR(true,20);
 //   armMotor.configClearPositionOnLimitF(false,20);
    armMotor.setSelectedSensorPosition(0);

    armMotor.configMotionCruiseVelocity(10000);
    armMotor.configMotionAcceleration(20000);
    armMotor.configMotionSCurveStrength(3);

    armMotor.selectProfileSlot(0, 0);

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kBrake);
    wofMotor.restoreFactoryDefaults();
    wofMotor.setIdleMode(IdleMode.kBrake);

    armsol1.set(sol1State);
    armsol2.set(sol2State);

    writeArmData();

}


public void setPosition(int pos){
    pidRunning=true;
    armMotor.selectProfileSlot(0, 0);
    int currentpos=armMotor.getSelectedSensorPosition(0);

    if (pos==1 ) {
        if (!lsf || currentpos<floorPos)
        armMotor.set(ControlMode.MotionMagic, floorPos);
    }

    else if (pos==2) {
        armMotor.set(ControlMode.MotionMagic, scorePos);
    }

    else if (pos==3) {
        if (!lsr || currentpos>intakePos)
        armMotor.set(ControlMode.MotionMagic, intakePos);
    }

    else if (pos==4) {
        armMotor.set(ControlMode.MotionMagic, topPos);
    }
    
}


public void manualSetPosition(double stickInput){
    pidRunning=false;
    if((lsf && stickInput<0 )|| (lsr && stickInput>0)) return;
    int currentPos = armMotor.getSelectedSensorPosition(0);
    double setPoint = currentPos+stickInput*10000;
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.PercentOutput, 0.5*stickInput);
//    armMotor.set(ControlMode.MotionMagic, setPoint);
}

public void shooterIn(){
    shooterMotor.set(1);
}

public void shooterOut(){
    shooterMotor.set(-1);
}

public void shooterStop(){
    shooterMotor.set(0);
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

public void writeArmData(){
//    SmartDashboard.putNumber("Arm/Arm Pos",armMotor.getSelectedSensorPosition(0));
 //   SmartDashboard.putNumber("Arm/Arm Error",armMotor.getClosedLoopError(0));
//  SmartDashboard.putNumber("Arm/Arm Output",armMotor.getMotorOutputPercent());
 //   SmartDashboard.putBoolean("Arm/Fwd Limit",lsf);
 //   SmartDashboard.putBoolean("Arm/Rev Limit",lsr);
 //   SmartDashboard.putNumber("Arm/Fwd LimitVolt",lsfVolt);
 //   SmartDashboard.putNumber("Arm/Rev LimitVolt",lsrVolt);
    SmartDashboard.putNumber("Arm/Rev Current",armMotor.getStatorCurrent());
    
  

}

public void brakeOn(){
    sol1State=true;sol2State=false;
    armsol1.set(sol1State);
    armsol2.set(sol2State);
}
public void brakeOff(){
    sol1State=false;sol2State=true;
    armsol1.set(sol1State);
    armsol2.set(sol2State);
}

public void toggleBrake(){
    sol1State=!sol1State;sol2State=!sol2State;
    armsol1.set(sol1State);
    armsol2.set(sol2State);
   }

public void brakeMonitor(){
    // check for limit switches
    lsfVolt =lsfAnalogIn.getVoltage(); 
    lsrVolt =lsrAnalogIn.getVoltage(); 
    double armVolt = armMotor.getMotorOutputPercent();

    if (lsfVolt>2.5 ) lsf=false;
    else {
        lsf=true;
        if (armVolt<0) {
            armMotor.set(ControlMode.PercentOutput, 0);
        }
        armMotor.setSelectedSensorPosition(0);

    }

    if (lsrAnalogIn.getVoltage()>2.5) lsr=false;
    else{         
        lsr=true;
        if (armVolt>0) armMotor.set(ControlMode.PercentOutput, 0);
    }


    // if brake is on and motor is driving, turn brake off
    if(Math.abs(armMotor.getMotorOutputVoltage())>0.06 && sol1State) {
        brakeOff();
    } 
    // if brake is off and motor stopped, turn brake on 
    else if (Math.abs(armMotor.getMotorOutputVoltage())<=0.06 && !sol1State ) {
        brakeOn();
        armMotor.set(ControlMode.PercentOutput, 0);
    }
}

public void armMotorStop(){
    armMotor.set(ControlMode.PercentOutput, 0);
}

} 