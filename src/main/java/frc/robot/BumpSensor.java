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
double vel,vel1,vel2;
int currentDist,dist1;
double motionTargetAngle;  
Accelerometer accelerometer = new BuiltInAccelerometer();
private Notifier _notifier;


BumpSensor(FalconDriveCTRE _drive, Stick _joystick){
    drive=_drive;
    joystick = _joystick;

    class PeriodicRunnable implements java.lang.Runnable {
        
        public void run() {

            currentDist=drive.br.getSelectedSensorPosition(0);
            if(currentDist<dist1)
                vel=vel1;
            else 
                vel=vel2;
            drive.br.set(ControlMode.Velocity, vel, DemandType.AuxPID, motionTargetAngle);
            drive.bl.follow(drive.br, FollowerType.AuxOutput1);


            if(accelerometer.getY()<-0.15 || 
                Math.abs(joystick.getRawAxis(1))>0.1 ){
                    drive.setMotors(0,0,ControlMode.Velocity);
                    stop();     
            }

        }
    }

    _notifier = new Notifier(new PeriodicRunnable());
}



public void run(double _vel1, double _vel2, double _dist1){
    Robot.runningPID=true;
    drive.setupTalonBump();
    motionTargetAngle=0;    
    dist1=(int)(_dist1*Constants.kSensorUnitsPerInch);
    System.out.println("Dist1 = "+dist1);
    vel1 = _vel1*Constants.k_InchPerSecToVelUnits;
    vel2 = _vel2*Constants.k_InchPerSecToVelUnits;
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





