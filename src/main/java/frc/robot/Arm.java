package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

 
public class Arm{
    TalonFX armMotor = new TalonFX(6);
    private double maxFF=0.1;
    private int topPos = 80000, intakePos=30000,scorePos=10000, floorPos = 0;
    private int floorFF,topFF, intakeFF,scoreFF;

Arm(){

    floorFF=(int) Math.cos( (floorPos/topPos)*(Math.PI/2) );
    scoreFF=(int) Math.cos( (scorePos/topPos)*(Math.PI/2) );
    intakeFF=(int) Math.cos( (intakePos/topPos)*(Math.PI/2) );
    topFF=(int) Math.cos( (topPos/topPos)*(Math.PI/2) );

    armMotor.configFactoryDefault();
    armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMotor.set(ControlMode.PercentOutput, 0);
    
    armMotor.config_kP(0,.15);
    armMotor.config_kD(0,0);

    armMotor.configClosedLoopPeakOutput(0, .5);

    
    armMotor.config_kP(1,.10);
    armMotor.config_kD(1,0);
    armMotor.configClosedLoopPeakOutput(1,0.4);

    armMotor.configClearPositionOnLimitF(true,20);


}


public void setPosition(int pos){
    if (pos==1) {
        armMotor.set(ControlMode.MotionMagic, floorPos , DemandType.ArbitraryFeedForward, floorFF);
    }
    else if (pos==2) {
        armMotor.set(ControlMode.MotionMagic, scorePos , DemandType.ArbitraryFeedForward, scoreFF);
    }
    else if (pos==3) {
        armMotor.set(ControlMode.MotionMagic, intakePos , DemandType.ArbitraryFeedForward, intakeFF);
    }
    else if (pos==4) {
        armMotor.set(ControlMode.MotionMagic, topPos , DemandType.ArbitraryFeedForward, topFF);
    }
    
}


public void manualSetPosition(double stickInput){
    int currentPos = armMotor.getSelectedSensorPosition(0);
    double ff = getFF(currentPos);
    double setPoint = currentPos+stickInput*1600;
    if (setPoint>topPos) setPoint=topPos;
    if (setPoint<floorPos) setPoint=floorPos;
    armMotor.set(ControlMode.Position, setPoint , DemandType.ArbitraryFeedForward, ff);
    if (armMotor.getSensorCollection().isFwdLimitSwitchClosed() == 1) armMotor.setSelectedSensorPosition(floorPos);
    if (armMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) armMotor.setSelectedSensorPosition(topPos);
}



private double getFF(int currentPos){
    double angle=(Math.PI/2.)*currentPos/(double)topPos;
    return Math.cos(angle)*maxFF;
}

} 