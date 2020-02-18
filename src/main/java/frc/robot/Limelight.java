package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

import edu.wpi.first.networktables.*;

public class Limelight {
    private boolean hasTarget = false;
    private boolean llDriving = false;
    double STEER_K = 0.08; // 0.03; how hard to turn toward the target
    double STEER_D = 0.17; // 0.03; how hard to turn toward the target
    double DRIVE_K = .045; // 0.26; how hard to drive fwd toward the target
    double DRIVE_D = 0.6; // 0.26; how hard to drive fwd toward the target
    double DESIRED_TARGET_AREA = 30; // 13.0; Area of the target when the robot reaches the wall
    double MAX_DRIVE = 0.5; // 0.7; Simple speed limit so we don't drive too fast
    double tx = 0, ty, ta = 0, tv;
    private Notifier _notifierStraight;
    FalconDriveCTRE drive;
    PathPlanner pathPlanner = new PathPlanner();
    MotionProfileCTRE mpctre;

    Limelight(Stick stick, FalconDriveCTRE _drive, MotionProfileCTRE _mpctre) {
        drive = _drive;
        mpctre = _mpctre;
  
        SmartDashboard.putNumber("Limelight/STEER_K", STEER_K);
        SmartDashboard.putNumber("Limelight/STEER_D", STEER_D);
        SmartDashboard.putNumber("Limelight/DRIVE_K", DRIVE_K);
        SmartDashboard.putNumber("Limelight/DRIVE_D", DRIVE_D);
        SmartDashboard.putNumber("Limelight/DESIRED AREA", DESIRED_TARGET_AREA);
        SmartDashboard.putNumber("Limelight/MAX_DRIVE", MAX_DRIVE);

        class StraightToTarget implements java.lang.Runnable {
            double measuredDist = ta, errorDist = ta, preverrorDist = 0, derrorDist = 0;
            double errorTargets = 0.5;
            double timeOnTarget;
            double errorAngle, preverrorAngle = 0, derrorAngle = 0;
            double forward, turn, leftinput, rightinput;
            double targetAngle;

            public void run() {
                getLimelightData();
                measuredDist = ta;
                errorAngle = tx;
                errorDist = DESIRED_TARGET_AREA - ta;
                derrorDist = errorDist - preverrorDist;
                derrorAngle = errorAngle - preverrorAngle;

                forward = DRIVE_K * errorDist + DRIVE_D * derrorDist;
                turn = STEER_K * errorAngle + STEER_D * derrorAngle;

                leftinput = forward + turn;
                rightinput = forward - turn;

                if (leftinput > MAX_DRIVE)
                    leftinput = MAX_DRIVE;
                if (leftinput < -MAX_DRIVE)
                    leftinput = -MAX_DRIVE;
                if (rightinput > MAX_DRIVE)
                    rightinput = MAX_DRIVE;
                if (rightinput < -MAX_DRIVE)
                    rightinput = -MAX_DRIVE;

                if (Math.abs(errorDist) <= errorTargets)
                    timeOnTarget += 0.02;
                else
                    timeOnTarget = 0.0;
                drive.setMotors(leftinput, rightinput);
                preverrorDist = errorDist;
                preverrorAngle = errorAngle;

                if (timeOnTarget > 0.5 || Math.abs(stick.getRawAxis(1)) > 0.1 || !hasTarget) {
                    stopStraight();
                }

            }
        }
        _notifierStraight = new Notifier(new StraightToTarget());
    }

    public void getLimelightData() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        SmartDashboard.putNumber("Limelight/tv", tv);
        SmartDashboard.putNumber("Limelight/tx", tx);
        SmartDashboard.putNumber("Limelight/ta", ta);
        hasTarget = true;
    }

    public void driveStraightToTarget() {
        STEER_K = SmartDashboard.getNumber("Limelight/STEER_K", 0);
        DRIVE_K = SmartDashboard.getNumber("Limelight/DRIVE_K", 0);
        STEER_D = SmartDashboard.getNumber("Limelight/STEER_D", 0);
        DRIVE_D = SmartDashboard.getNumber("Limelight/DRIVE_D", 0);
        DESIRED_TARGET_AREA = SmartDashboard.getNumber("Limelight/DESIRED AREA", 0);
        MAX_DRIVE = SmartDashboard.getNumber("Limelight/MAX_DRIVE", 0);

        if (tv < 1.0) {
            hasTarget = false;
            llDriving = false;
        } else {
            Robot.runningPID=true;
            drive.resetGyro();
            hasTarget = true;
            llDriving = true;
            _notifierStraight.startPeriodic(0.01);
        }
    }

    public void stopStraight() {
        Robot.runningPID = false;
        llDriving = false;
        _notifierStraight.stop();
    }

    public void driveProfileToTarget() throws IOException {
    double targetAngle=tx*1.1+drive.getAngle(); // Need to work this out
    
    if (tv < 1.0) {
      hasTarget = false;
     llDriving=false;
    }
    else{
    Robot.runningPID=true;
    hasTarget=true;    
    llDriving=true;
    double distance=1/(ta+0.0001);
    double robotAngle=drive.getAngle();
    double visionTargetAngle=1.1*tx;   
    double alpha = (90-(visionTargetAngle-robotAngle))*Math.PI/180;;
    double dx =  distance*Math.cos(alpha);
    double dy =  distance*Math.sin(alpha);
    pathPlanner.makePath(0, 0, robotAngle, dx, dy, 0.0, 48, 96, 180);
//    mpctre.runProfile(pathPlanner.stream);
    


    }
}



public boolean isRunning(){
    return llDriving;
}


}