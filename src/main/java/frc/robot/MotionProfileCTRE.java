package frc.robot;

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
    Arm arm;
    Stick stick;
    int i=0,act1=0,act2=0;
    MotionProfileStatus status = new MotionProfileStatus();

    MotionProfileCTRE(FalconDriveCTRE _drive,Stick _stick, Arm _arm){
        drive = _drive;
        stick = _stick;
        arm = _arm;

        class PeriodicRunnable implements java.lang.Runnable {
            
            public void run() {
                // run actions            
                act1=profile.action1[i];
                if (act1!=0){
                    System.out.println("act1 = "+act1);
                    switch (act1) {
                        case 1:
                            arm.shooterOut();
                        break;
                        case 2:
                            arm.shooterIn();
                        break;
                        case 3:
                            arm.shooterStop();
                        break;

                }
            }
                act2=profile.action2[i];
                if (act2!=0){
                    System.out.println("act2 = "+act2);
                    switch (act2) {
                        case 1:
                            arm.setPosition(0);
                        break;
                        case 2:
                            arm.setPosition(2);
                        break;
                        case 3:
                            arm.setPosition(3);
                        break;
                }
            }
            arm.brakeMonitor();


                // print status every 10 cycles
                if(i<(profile.size-2) )i++;
                iend=i;
                count++;
                if(count%10==0){
                     System.out.println(

                     "ATP_0 "+drive.br.getActiveTrajectoryPosition(0)+"  "+
                     "CLE_0 "+drive.br.getClosedLoopError(0)+"  "+
                     "ATP_1 "+drive.br.getActiveTrajectoryPosition(1)+"  "+
                     "CLE_1 "+drive.br.getClosedLoopError(1));
                    count=0;
                }

                // Abort if joystick is pushed
                if ( drive.br.isMotionProfileFinished() || stick.isPushed()) {
                    Robot.runningPID=false;
                    stop();
                }    

            }            
        }
        notifier = new Notifier(new PeriodicRunnable());
    }
    

//    public void runProfile(Profile _profile) {
    public void runProfile(String name) {
//        profile = _profile;
        profile = new Profile(name,2);
        iend=0;
        count=0;
        i=0;act1=0;
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



