package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

 
public class Elevator{
    TalonFX elevator = new TalonFX(5);
    private int topPos = 460000;
    private int hangPos = 360000; 
    private int botPos = 0;
    private int setPoint=0;
    int  calibrated =0;
Elevator(){
    elevator.configFactoryDefault();
    elevator.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    elevator.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    elevator.set(ControlMode.PercentOutput, 0);
//    elevator.configForwardSoftLimitEnable(true);
//    elevator.configReverseSoftLimitEnable(true);
//    elevator.configForwardSoftLimitThreshold(3000);
//    elevator.configReverseSoftLimitThreshold(0);
    elevator.config_kP(0,.06);
    elevator.config_kD(0,0);
    elevator.config_kF(0,0.056);
    elevator.configClosedLoopPeakOutput(0, 1.0);

    
    elevator.config_kP(1,.05);
    elevator.config_kD(1,0);
    elevator.config_kF(1,0.04);

    elevator.configClosedLoopPeakOutput(1,0.40);

    elevator.configClearPositionOnLimitR(true,20);

    elevator.configMotionCruiseVelocity(18000);
    elevator.configMotionAcceleration(36000);


}


public void setPosition(){
    if(calibrated==2){
    if (setPoint==botPos) {
        setPoint=topPos;
        elevator.selectProfileSlot(0, 0);
        elevator.set(ControlMode.Position, setPoint);
    }
    else{
        setPoint =botPos;
        elevator.selectProfileSlot(1, 0);
        elevator.set(ControlMode.Position, setPoint);

    } 
}
}


//  bring elevator to top
public void up(){
    if(calibrated==2){
        setPoint=topPos;
        elevator.selectProfileSlot(0, 0);
        elevator.set(ControlMode.MotionMagic, setPoint);
    }
}

//  bring elevator down 1 level
public void down(){
    if(calibrated==2){
        if (setPoint==topPos) setPoint=hangPos;
        else setPoint=botPos;
        elevator.selectProfileSlot(1, 0);
        elevator.set(ControlMode.MotionMagic, setPoint);
        System.out.println("SetPoint = "+setPoint);
    }
}


public void calibrate(){
    if(calibrated==0){
    calibrated=1;  // running calibration
    System.out.println("Start calibtationA");

    (new Thread() {
        public void run() {
            System.out.println("Start calibtation");
            while(elevator.getSensorCollection().isRevLimitSwitchClosed() ==0){
                System.out.println("Calibrating");
                elevator.set(ControlMode.PercentOutput,-0.2);        
            }
            elevator.set(ControlMode.PercentOutput,0);
            elevator.setSelectedSensorPosition(0);
            calibrated=2;      // done calibrating
            System.out.println("Done Calibrating") ;
        }
       }).start();
    }


 
}

} 