package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

//angle:  kp=0.0075,  kI=0.00075, kiZ = 2, Max = 0.8  
// position:  all zero
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class PIDRotate{
FalconDriveCTRE drive;
Stick joystick;
double timeOnTarget;
double timeOnTargetGoal=Constants.timeOnTargetGoal;
double errorAllowable=Constants.errorAllowable;
double measured_angle;
double measured_dist;
double targetAngle;
double inp_min=Constants.minTurnInp;

private Notifier _notifier;


double motionTargetAngle;  

PIDRotate(FalconDriveCTRE _drive, Stick _joystick){
    drive=_drive;
    joystick = _joystick;

    class PeriodicRunnable implements java.lang.Runnable {

        double errora, preverrora=0, derrora=0, interrora=0;
        double errorp, preverrorp=0, derrorp=0, interrorp=0;
        double forward, turn,leftinput,rightinput;
        
        public void run() {
            measured_angle=drive.getAngle();
            measured_dist=0.5*(drive.getlpos() +drive.getrpos());
         
            errora=targetAngle-measured_angle;
            errorp=0-measured_dist;

            derrora=errora-preverrora;
            derrorp=errorp-preverrorp;
 
            if(Math.abs(errora)<=Constants.angleRot.kIz) 
                interrora += errora;
            else 
                interrora=0;
            
            if(Math.abs(errorp)<=Constants.pos.kIz) 
                interrorp += errorp;
            else 
                interrorp=0;

            
            turn=Constants.angleRot.kP*errora + Constants.angleRot.kD*derrora + Constants.angleRot.kI*interrora+inp_min;               
            forward=Constants.pos.kP*errorp + Constants.pos.kD*derrorp + Constants.pos.kI*interrorp;

            leftinput=forward-turn;rightinput=forward+turn;

// cap the motor input to max allowable, and don't let it fall below inp_min threshold (no stalling)            
            if(leftinput> Constants.angleRot.kMax)leftinput=Constants.angleRot.kMax;
            else if(leftinput< -Constants.angleRot.kMax)leftinput=-Constants.angleRot.kMax;
            else if(Math.abs(leftinput)<inp_min) leftinput=inp_min*Math.signum(leftinput);

            if(rightinput> Constants.angleRot.kMax)rightinput=Constants.angleRot.kMax;
            else if(rightinput< -Constants.angleRot.kMax)rightinput=-Constants.angleRot.kMax;
            else if(Math.abs(rightinput)<inp_min) rightinput=inp_min*Math.signum(rightinput);
    
            if (Math.abs(errora)<=errorAllowable)
                timeOnTarget +=0.02;
            else 
                timeOnTarget=0.0;          
//            drive.setMotors(leftinput, rightinput);  
            drive.setMotors(leftinput,rightinput,ControlMode.Velocity);
            preverrorp=errorp;
            preverrora=errora;
            writePIDVars(measured_angle, errora);

            if(timeOnTarget>timeOnTargetGoal || 
                Math.abs(joystick.getRawAxis(1))>0.1 ){
                   stop();     
            }

        }
    }

    _notifier = new Notifier(new PeriodicRunnable());
}



public void run(double _targetAngle, boolean reset){
    targetAngle=_targetAngle;
    if (reset) resetSensors();
    Constants.readGains();
    drive.setVelocityGains();
    Robot.runningPID=true;
    _notifier.startPeriodic(0.01);
}


public void stop(){
    Robot.runningPID=false;
    _notifier.stop();
}


public void resetSensors(){
drive.resetEncoders();
drive.gyro.setYaw(0);
}

public void writePIDVars(double measured, double error){
        SmartDashboard.putNumber("Measured", measured);
        SmartDashboard.putNumber("Error", error);
    }

}





