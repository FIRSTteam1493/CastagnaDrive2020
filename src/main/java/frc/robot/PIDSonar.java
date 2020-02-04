package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class PIDSonar{
FalconDriveCTRE drive;
PigeonIMU gyro;
Sonar sonar;
Stick joystick;
double timeOnTarget=2;
double measured_angle,measured_sonar;
double measured_distl,measured_distr, measured_dist;
double targetDist;

private Notifier _notifier;


double motionTargetAngle;  

PIDSonar(FalconDriveCTRE _drive, Stick _joystick){
    drive=_drive;
    joystick = _joystick;

    class PeriodicRunnable implements java.lang.Runnable {
        double errors, preverrors=0, derrors=0, interrors=0;
        double errorTargets=0.5;
        double errora, preverrora=0, derrora=0, interrora=0;
        double forward, turn,leftinput,rightinput;
        double targeta=drive.getAngle();

        
        public void run() {
            measured_sonar=sonar.getDistance();
            measured_angle=drive.getAngle();
            errors=measured_sonar-targetDist;
            errora=targeta-measured_angle;
            derrors=errors-preverrors;
            derrora=errora-preverrora;
            if(Math.abs(errors)<=Constants.sonar.kIz) 
                interrors += errors;
            else 
                interrors=0;

            if(Math.abs(errora)<=Constants.angleMP.kIz) 
                interrora += errora;
            else 
                interrora=0;
               
            forward=Constants.sonar.kP*errors + Constants.sonar.kD*derrors + Constants.sonar.kI*interrors;
            turn=Constants.angleMP.kP*errora + Constants.angleMP.kD*derrora + Constants.angleMP.kI*interrora;
            leftinput=forward+turn;rightinput=forward-turn;
            leftinput= Math.max(Constants.sonar.kMax, Math.min(leftinput,
            Constants.sonar.kMax));
            rightinput= Math.max(Constants.sonar.kMax, Math.min(rightinput,Constants.sonar.kMax));
            if (Math.abs(errors)<=errorTargets)
                timeOnTarget +=0.02;
            else 
                timeOnTarget=0.0;          
            drive.setMotors(leftinput, rightinput,ControlMode.Velocity);  
            preverrors=errors;
            preverrora=errora;

            writePIDVars(measured_angle, errora);

            if(timeOnTarget<Constants.timeOnTargetGoal || 
                Math.abs(joystick.getRawAxis(1))>0.1 ){
                   stop();     
            }

        }
    }

    _notifier = new Notifier(new PeriodicRunnable());
}



public void run(double _targetDist){
    targetDist=_targetDist;
    resetSensors();
    Constants.readGains();
    drive.setVelocityGains();
    Robot.runningPID=true;
    _notifier.startPeriodic(0.01);
}


public void stop(){
    _notifier.stop();
    Robot.runningPID=false;
}


public void resetSensors(){
drive.resetEncoders();
gyro.setYaw(0);
}

public void writePIDVars(double measured, double error){
        SmartDashboard.putNumber("Measured", measured);
        SmartDashboard.putNumber("Error", error);
    }

}





