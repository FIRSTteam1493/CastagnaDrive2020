package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class BumpSensor{
FalconDriveCTRE drive;
Stick joystick;
double measured_angle;
double vel;
Accelerometer accelerometer = new BuiltInAccelerometer();

private Notifier _notifier;

int count=0;
double motionTargetAngle;  

BumpSensor(FalconDriveCTRE _drive, Stick _joystick){
    drive=_drive;
    joystick = _joystick;

    class PeriodicRunnable implements java.lang.Runnable {
        
        public void run() {
            drive.br.set(ControlMode.Velocity, vel, DemandType.AuxPID, motionTargetAngle);
            drive.bl.follow(drive.br, FollowerType.AuxOutput1);

            if(accelerometer.getY()<-0.15 || 
                Math.abs(joystick.getRawAxis(1))>0.1 ){
                    drive.br.set(ControlMode.Velocity, 0);
                    drive.bl.set(ControlMode.Velocity, 0);
                    
                   stop();     
            }

        }
    }

    _notifier = new Notifier(new PeriodicRunnable());
}



public void run(double _vel){
    Robot.runningPID=true;
    drive.setupTalonBump();
    motionTargetAngle=0;
    vel = vel*Constants.k_InchPerSecToVelUnits;
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





