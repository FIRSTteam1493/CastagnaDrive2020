package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 
public class Arm{
    private TalonFX armMotor = new TalonFX(6);
    private int topPos = 86000, intakePos=86000,scorePos=86000, floorPos = 0;
    private CANSparkMax shooterMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax wofMotor = new CANSparkMax(2, MotorType.kBrushless);

    private Solenoid armsol1 = new Solenoid(3);
    private Solenoid armsol2 = new Solenoid(6);
    // On Position:   solState1 = true       solState2 = false
    // Off Position:   solState1 = false     solState2 = true
    // start with brake on
    private boolean sol1State = true;
    private boolean sol2State = false;

Arm(){

    armMotor.configFactoryDefault();
    armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    System.out.println("lsf = "+    
    armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen));

    System.out.println("lsr = "+    
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen));
    
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

    armMotor.configClearPositionOnLimitF(true,20);
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
    armMotor.selectProfileSlot(0, 0);

    if (pos==1) {
        armMotor.set(ControlMode.MotionMagic, floorPos);

    }
    else if (pos==2) {
        armMotor.set(ControlMode.MotionMagic, intakePos);
    }
    else if (pos==3) {
        armMotor.set(ControlMode.MotionMagic, scorePos);
    }
    else if (pos==4) {
        armMotor.set(ControlMode.MotionMagic, topPos);
    }
    
}


public void manualSetPosition(double stickInput){
    int currentPos = armMotor.getSelectedSensorPosition(0);
    double setPoint = currentPos+stickInput*10000;
//    if (setPoint>topPos) setPoint=topPos;
  //  if (setPoint<floorPos) setPoint=floorPos;
  // use profile slot 1
    armMotor.selectProfileSlot(1,0);
//    armMotor.set(ControlMode.PercentOutput, 0.5*stickInput);
    armMotor.set(ControlMode.MotionMagic, setPoint);
//    if (armMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1) armMotor.setSelectedSensorPosition(floorPos);
//    if (armMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) armMotor.setSelectedSensorPosition(0);
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
    SmartDashboard.putNumber("Arm/Arm Pos",armMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Arm/Arm Error",armMotor.getClosedLoopError(0));
    SmartDashboard.putNumber("Arm/Arm Output",armMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Arm/Fwd Limit",armMotor.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Arm/Rev Limit",armMotor.getSensorCollection().isRevLimitSwitchClosed());
    
  

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





} 