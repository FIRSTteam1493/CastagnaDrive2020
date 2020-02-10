package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class PIDStraightMagic{
FalconDriveCTRE drive;
Stick joystick;
double timeOnTarget=0;
double measured_angle,measured_sonar;
double measured_distl,measured_distr, measured_dist;
double targetPos;

private Notifier _notifier;

int count=0;
double motionTargetAngle;  

PIDStraightMagic(FalconDriveCTRE _drive, Stick _joystick){
    drive=_drive;
    joystick = _joystick;

    class PeriodicRunnable implements java.lang.Runnable {
        double errorTargets=30, errorp;
        
        public void run() {
            drive.br.set(ControlMode.MotionMagic, targetPos, DemandType.AuxPID, motionTargetAngle);
            drive.bl.follow(drive.br, FollowerType.AuxOutput1);
            measured_dist = drive.br.getSelectedSensorPosition(0);
            errorp=targetPos-measured_dist;

            if (Math.abs(errorp)<=errorTargets)
                timeOnTarget +=0.02;
            else 
                timeOnTarget=0.0;    

            writePIDVars(measured_dist, errorp);
            count++;
            if(count%10==0) {System.out.println(
                "CLT_0 "+drive.br.getClosedLoopTarget(0)+"   "+
                "CLE_0 "+drive.br.getClosedLoopError(0)+"   "+
                "CLT_1 "+drive.br.getClosedLoopTarget(1)+"  "+
                "CLE_1 "+drive.br.getClosedLoopError(1));
                count=0;
            }

            if(timeOnTarget>1 || 
                Math.abs(joystick.getRawAxis(1))>0.1 ){
                   stop();     
            }

        }
    }

    _notifier = new Notifier(new PeriodicRunnable());
}



public void run(double _targetPos, int vel, int acc){
    Robot.runningPID=true;
    drive.setupTalonTeleop(); 
    drive.resetEncoders();
    drive.resetGyro();
    drive.setupTalonMotionMagicStraight();
    drive.br.configMotionAcceleration(acc, 20);
    drive.br.configMotionCruiseVelocity(vel, 20);
    drive.br.configMotionSCurveStrength(4, 20);
    targetPos=_targetPos*Constants.kSensorUnitsPerInch;
    System.out.println("A1 "+targetPos);
    _notifier.startPeriodic(0.01);
}


public void stop(){
    _notifier.stop();
    Robot.runningPID=false;
    drive.setupTalonTeleop();
}


public void writePIDVars(double measured, double error){
        SmartDashboard.putNumber("Measured", measured);
        SmartDashboard.putNumber("Error", error);
    }

}





