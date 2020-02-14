package frc.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import edu.wpi.first.wpilibj.Notifier;

public class MotionProfileCTRE{
    static int iend;
    double time1;
    int count=0;
    Notifier notifier;
    Profile profile;
    FalconDriveCTRE drive;
    Stick stick;
    MotionProfileStatus status = new MotionProfileStatus();

    MotionProfileCTRE(FalconDriveCTRE _drive,Stick _stick){
        drive = _drive;
        stick = _stick;

        class PeriodicRunnable implements java.lang.Runnable {
            int i=0,act1=0;
// run actions and report on status            
            public void run() {
                if ( profile.action1[i]!=act1) System.out.println("i "+i+"   act1 "+act1);
                act1=profile.action1[i];
                if(i<(profile.size-2) )i++;
                iend=i;
                count++;
                if(count%10==0){
                     System.out.println(
                     "ATP_0 "+drive.br.getActiveTrajectoryPosition(0)+"   "+
                     "CLE_0 "+drive.br.getClosedLoopError(0)+"   "+
                     "ATP_1 "+drive.br.getActiveTrajectoryPosition(1)+"  "+
                     "CLE_1 "+drive.br.getClosedLoopError(1));
                    count=0;
                }


                if ( drive.br.isMotionProfileFinished() || stick.isPushed()) {
                    Robot.runningPID=false;
                    stop();
                }    

            }            
        }
        notifier = new Notifier(new PeriodicRunnable());
    }

    public void runProfile(Profile _profile) {
        profile = _profile;
        iend=0;
        count=0;
        drive.setupTalonTeleop();
        drive.resetEncoders();
        drive.resetGyro();    
        drive.setupTalonMP();
        time1=System.nanoTime();
        Robot.runningPID=true;
        drive.br.startMotionProfile(profile.stream, 10, ControlMode.MotionProfileArc);
        drive.bl.follow(drive.br, FollowerType.AuxOutput1);
        notifier.startPeriodic(0.01);
    }


    public void stop(){
    notifier.stop();
    drive.br.set(ControlMode.Velocity,0.0);
    drive.bl.set(ControlMode.Velocity,0.0);
    drive.setupTalonTeleop();
    System.out.println("Time elapsed = "+(System.nanoTime()-time1)/1000000. );
    System.out.println("iend = "+iend);


    }
   
}   



