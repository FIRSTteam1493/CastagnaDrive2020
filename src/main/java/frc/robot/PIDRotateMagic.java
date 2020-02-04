package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class PIDRotateMagic{
FalconDriveCTRE drive;
PigeonIMU gyro;
Stick joystick;
double timeOnTarget=1;
double measured_angle,measured_sonar;
double measured_distl,measured_distr, measured_dist;
double targetAngle;

private Notifier _notifier;


PIDRotateMagic(FalconDriveCTRE _drive, Stick _joystick){
    drive=_drive;
    joystick = _joystick;

    class PeriodicRunnable implements java.lang.Runnable {
        double errorTargets=10, errora, erroraux;
        
        public void run() {
            drive.br.set(ControlMode.MotionMagic, targetAngle, DemandType.AuxPID, 0.0);
            drive.bl.follow(drive.br, FollowerType.AuxOutput1);
            erroraux=drive.br.getSelectedSensorPosition(1);
            errora=Math.abs(targetAngle-drive.getAngle()*8192/360.);
            if (Math.abs(errora)<=errorTargets)
                timeOnTarget +=0.02;
            else 
                timeOnTarget=0.0;    

            writePIDVars(measured_angle, errora,erroraux);

            if(timeOnTarget>1 || 
                Math.abs(joystick.getRawAxis(1))>0.1 ){
                   stop();     
            }


        }
    }

    _notifier = new Notifier(new PeriodicRunnable());
}



public void run(double _targetAngle){
    targetAngle=_targetAngle*Constants.k_gyroUnitsPerDegree;
    Robot.runningPID=true;
    drive.setupTalonTeleop(); 
    drive.resetEncoders();
    drive.resetGyro();
    drive.setupTalonMotionMagicRotate();       
    System.out.println("A1 "+targetAngle);
    _notifier.startPeriodic(0.01);
}


public void stop(){
    _notifier.stop();
    Robot.runningPID=false;
    drive.setupTalonTeleop();
}


public void resetSensors(){
drive.resetEncoders();
gyro.setYaw(0);
}

public void writePIDVars(double measured, double error,double erroraux){
        SmartDashboard.putNumber("Measured", measured);
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("ErrorAux", erroraux);
    }

}





