package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
    private double maxFF=0.00;
    private int topPos = 90000, intakePos=30000,scorePos=75000, floorPos = 0;
    private double floorFF,topFF, intakeFF,scoreFF;    
    private CANSparkMax shooterMotor = new CANSparkMax(3, MotorType.kBrushless);
    private Solenoid armsol1 = new Solenoid(0);
    private Solenoid armsol2 = new Solenoid(1);
    // On Position:   solState1 = true       solState2 = false
    // Off Position:   solState1 = false     solState2 = true
    // start with brake on
    private boolean sol1State = true;
    private boolean sol2State = false;

Arm(){

    floorFF=getFF(floorPos);
    scoreFF=getFF(scorePos);
    intakeFF=getFF(intakePos);
    topFF= getFF(topPos);

    armMotor.configFactoryDefault();
    armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMotor.set(ControlMode.PercentOutput, 0);
    
// slot 0 for magic motion by button    
    armMotor.config_kP(0,.5);
    armMotor.config_kD(0,.2);
    armMotor.configClosedLoopPeakOutput(0, .5);
    
// slot 1 for manual control    
    armMotor.config_kP(1,.4);
    armMotor.config_kD(1,0.1);
    armMotor.configClosedLoopPeakOutput(1,0.4);

    armMotor.configClearPositionOnLimitF(true,20);
    armMotor.setSelectedSensorPosition(0);

    armMotor.configMotionCruiseVelocity(15000);
    armMotor.configMotionAcceleration(30000);

    armMotor.selectProfileSlot(0, 0);

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kBrake);

    armsol1.set(sol1State);
    armsol2.set(sol1State);

    writeArmData();
    SmartDashboard.putNumber("Arm/MaxFF", maxFF);

}


public void setPosition(int pos){
    maxFF = SmartDashboard.getNumber("Arm/MaxFF", 0);
    armMotor.selectProfileSlot(0, 0);

    if (pos==1) {
        floorFF=getFF(floorPos);
        armMotor.set(ControlMode.MotionMagic, floorPos , DemandType.ArbitraryFeedForward, floorFF);
        System.out.println("floorFF = "+floorFF);
    }
    else if (pos==2) {
        scoreFF=getFF(scorePos);
        armMotor.set(ControlMode.MotionMagic, scorePos , DemandType.ArbitraryFeedForward, scoreFF);
        System.out.println("scoreFF = "+scoreFF);
        SmartDashboard.putNumber("Arm/Score FF", scoreFF);
    }
    else if (pos==3) {
        intakeFF=getFF(intakePos);
        armMotor.set(ControlMode.MotionMagic, intakePos , DemandType.ArbitraryFeedForward, intakeFF);
        System.out.println("intakeFF = "+intakeFF);
    }
    else if (pos==4) {
        topFF= getFF(topPos);
        armMotor.set(ControlMode.MotionMagic, topPos , DemandType.ArbitraryFeedForward, topFF);
        System.out.println("topFF = "+topFF);
    }
    
}


public void manualSetPosition(double stickInput){
    int currentPos = armMotor.getSelectedSensorPosition(0);
    double ff = getFF(currentPos);
    double setPoint = currentPos+stickInput*10000;
    if (setPoint>topPos) setPoint=topPos;
  //  if (setPoint<floorPos) setPoint=floorPos;
  // use profile slot 1
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, setPoint , DemandType.ArbitraryFeedForward, ff);
//    if (armMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1) armMotor.setSelectedSensorPosition(floorPos);
    if (armMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) armMotor.setSelectedSensorPosition(0);
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
    armsol2.set(sol1State);
}
public void brakeOff(){
    sol1State=false;sol2State=true;
    armsol1.set(sol1State);
    armsol2.set(sol1State);
}

public void toggleBrake(){
    sol1State=!sol1State;sol2State=!sol2State;
    armsol1.set(sol1State);
    armsol2.set(sol1State);
}

public void brakeMonitor(){
    // if brake is on and motor is driving, turn brake off
    if(armMotor.getMotorOutputVoltage()>0.06 && sol1State) brakeOff();    
    // if brake is off and motor stopped, turn brake on 
    else if (armMotor.getMotorOutputVoltage()<=0.06 && !sol1State ) {
        brakeOn();
        armMotor.set(ControlMode.PercentOutput, 0);
    }
}


private double getFF(int currentPos){
    double angle=(Math.PI/2.)*currentPos/(double)topPos;
    return Math.cos(angle)*maxFF;
}





} 