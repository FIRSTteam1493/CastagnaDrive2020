package frc.robot;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class Auto{
    FalconDriveCTRE drive;
    Stick joy0;
    Arm arm;
    MotionProfileCTRE mpctre;
    PIDRotate pidRotate;  
    PIDRotateMagic pidRotateMagic;  
    PIDStraightMagic pidStraightMagic; 
     BumpSensor bump;
    Profile straight60_48,straightarc60_48;
    Profile shoot3_side,trench_shoot5;



    Auto(FalconDriveCTRE _drive, Stick _joy0, Arm _arm, MotionProfileCTRE _mpctre){
        drive = _drive;
        arm = _arm;
        mpctre = _mpctre;
        joy0=_joy0;

        pidRotate= new PIDRotate(drive,joy0);
        pidRotateMagic = new PIDRotateMagic(drive,joy0);
        bump = new BumpSensor(drive, joy0);
        pidStraightMagic= new PIDStraightMagic(drive,joy0);

        shoot3_side = new Profile("/home/lvuser/profile_side_shoot3.profile",2);   
        trench_shoot5 = new Profile("/home/lvuser/profile_trench_shoot5.profile",2);   
        straightarc60_48 = new Profile("/home/lvuser/profile_straightArc60_48.profile",2);   
        straight60_48 = new Profile("/home/lvuser/profile_straight60.profile",2);   
     //   straightarc120_48 = new Profile("/home/lvuser/profile_straightArc120_48.profile",2);   
    //    straightarc120_96 = new Profile("/home/lvuser/profile_straightArc120_96.profile",2);   
    //    wof_goal = new Profile("/home/lvuser/profile_wof_goal.profile",2);   
    
    }

    public void runAuto(String m_autoSelected){
        System.out.println("flag A");
        switch (m_autoSelected) {
            case "do_nothing":
                doNothing();
            break;
            case "shoot3_straight":
                shoot3Straight();
            break;
            case "shoot3_side":
            System.out.println("flag B");
                shoot3Side();    
            break;                
            case "shoot3_side_delay":
                shoot3SideDelay();    
            break;               
            case "push_shoot3":
                pushShoot3();
            break;
            case "trench_shoot5":
                trenchShoot5(); 
            break;                           
            case "backup_pass":
                mpctre.runProfile(straightarc60_48); 
            break;                           
            case "Other":
            mpctre.runProfile(straight60_48);    
        break;                           
            default:
              // Put default auto code here
          break;
      }
    }


    private void doNothing()
    {        
    }

    private void shoot3Straight()
    {   
        drive.setupTalonBump();
        arm.setPosition(2);
        bump.run(0.5,.35,48);
        arm.shooterOut();
        Timer.delay(0.5);
        arm.shooterStop();
        drive.br.setSelectedSensorPosition(0);
        drive.setupTalonMotionMagicStraight();
        pidStraightMagic.run(-24, 6000, 12000);
 //       pidRotate.run(-60, false);
        drive.setupTalonTeleop();
        drive.setMotors(0, 0, ControlMode.Velocity);
}

private void shoot3Side()
{   
   mpctre.runProfile(shoot3_side);
}

private void shoot3SideDelay()
{   
    Timer.delay(5);
    shoot3Side();
}



private void pushShoot3()
{
    drive.setupTalonMotionMagicStraight();
    pidStraightMagic.run(-24, -5000, -10000);   
    drive.setMotors(0, 0, ControlMode.Velocity);
    drive.setupTalonBump();
    arm.setPosition(2);
    bump.run(48,24,94);
    arm.shooterOut();
    Timer.delay(0.5);
    arm.shooterStop();
    drive.br.setSelectedSensorPosition(0);
    drive.setupTalonMotionMagicStraight();
    pidStraightMagic.run(-24, -6000, -12000);
    pidRotate.run(-60, false);
    drive.setMotors(0, 0, ControlMode.Velocity);
}

private void trenchShoot5()
{   
   mpctre.runProfile(trench_shoot5);
}


}