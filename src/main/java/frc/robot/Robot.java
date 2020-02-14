// to do  - loop for motionprofileCTRE should be in autoperiodic or teleopperiodic
// edit smartdash
// Comment 2/8
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANDigitalInput;
import org.opencv.core.Mat;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.opencv.videoio.VideoCapture;
//import badlog.lib.BadLog;

public class Robot extends TimedRobot {
    static boolean runningPID = false;
    public static boolean FX = true;
    Constants constants = new Constants();
//    BadLog log;
    VideoCapture camera;
    GripPipeline grip = new GripPipeline();
    Sonar sonar = new Sonar(1);
 //   SonarDigital sonarDigital = new SonarDigital();
    Stick joy0 = new Stick(0);
    FalconDriveCTRE drive = new FalconDriveCTRE();  
    Elevator elevator = new Elevator();
    MotionProfileCTRE mpctre = new MotionProfileCTRE(drive, joy0);
    Limelight limelight = new Limelight(joy0, drive, mpctre);
    PIDRotate pidRotate = new PIDRotate(drive,joy0);  
    PIDRotateMagic pidRotateMagic = new PIDRotateMagic(drive,joy0);  
    PIDStraightMagic pidStraightMagic = new PIDStraightMagic(drive,joy0);
    PIDSonar pidSonar = new PIDSonar(drive,joy0);
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
    double[] ypr_deg={0,0,0};
    CANDigitalInput reverseLimit;
//    ColorSensor colorSensor = new ColorSensor();
    Mat frame = new Mat();
    int counter=0;

    Profile straight60_48,straightarc60_48, straightarc120_48, straightarc120_96, wof_goal;

  @Override
  public void robotInit() {
    SmartDashboard.putString("Messages", "Hello");
    Constants.writeGains();
    camera = new VideoCapture(0);

    m_chooser.setDefaultOption("Rotate90", "Rotate90");
    m_chooser.addOption("Rotate180", "Rotate180");
    m_chooser.addOption("Straight120_48ips", "Straight120_48ips");
    m_chooser.addOption("Straight120_96ips", "Straight120_96ips");
    m_chooser.addOption("Profile_straight60_48", "Profile_straight60_48");
    m_chooser.addOption("Profile_straightarc60_48", "Profile_straightarc60_48");
    m_chooser.addOption("Profile_straightarc120_48", "Profile_straightarc120_48");
    m_chooser.addOption("Profile_straightarc120_96", "Profile_straightarc120_96");
    m_chooser.addOption("Profile_outback_96", "Profile_outback_96");
    m_chooser.addOption("Profile_WOF_Goal", "Profile_WOF_Goal");

    SmartDashboard.putData("Auto choices", m_chooser);
/*
    log = BadLog.init("/home/lvuser/test.bag");{
    BadLog.createValue("Date", " "+Timer.getFPGATimestamp());
    BadLog.createTopic("Vel_L", "rpm", ()->drive.getlvel()  );
    BadLog.createTopic("Vel_R", "rpm", ()->drive.getrvel()  );
    BadLog.createTopic("Input_L", "units", ()->drive.getLeftInput()  );
    BadLog.createTopic("Input_R", "units", ()->drive.getRightInput()  );
    BadLog.createTopic("Curr_L", "amps", ()->drive.bl.getStatorCurrent()  );
    BadLog.createTopic("Curr_R", "amps", ()->drive.br.getStatorCurrent()  );
        
}

    log.finishInitialization();
*/
    straight60_48 = new Profile("/home/lvuser/profile_straight60_48.profile",2);   
    straightarc60_48 = new Profile("/home/lvuser/profile_straightArc60_48.profile",2);   
 //   straightarc120_48 = new Profile("/home/lvuser/profile_straightArc120_48.profile",2);   
//    straightarc120_96 = new Profile("/home/lvuser/profile_straightArc120_96.profile",2);   
//    wof_goal = new Profile("/home/lvuser/profile_wof_goal.profile",2);   

}


  @Override
  public void autonomousInit() {
      drive.resetEncoders();
      drive.resetGyro();
  }


  @Override
  public void autonomousPeriodic() {
  }

  
  @Override
  public void teleopInit() {
  //    elevator.calibrate();
  }

  @Override
  public void teleopPeriodic() {

    joy0.readStick();
    if (elevator.calibrated==0 ) {
        System.out.println("Start calibtation0");

        elevator.calibrate();}
// write position and velocity to smartdashboard         

    //  Choose a  non-drive actions
    // Button 1 - set motor parameters
    //  Button 2 - Read Color Sensor;
    // Button 3 - process vision pipeline

    if(joy0.getButton(1) && !joy0.getPrevButton(1) ){
        Constants.readGains();
        drive.setVelocityGains();
        drive.resetEncoders();
        drive.resetGyro();
        SmartDashboard.putString("Messages", "V gains set");
    }

    else if (joy0.getButton(2)) {
//        colorSensor.getColor();
    }

    else if (joy0.getButton(3) && !joy0.getPrevButton(3) ){
        camera.read(frame);
        grip.process(frame);
    }
    else if (joy0.getButton(9) && !joy0.getPrevButton((9)) ){
        if(elevator.calibrated==2) elevator.setPosition();
    }



     // Choose a drive actions  - either run a PID or drive from stick   
    // Button 5 - run Rotate PID
    // Button 6 - run Straight PID
    // Button 7 - position control 
    // Button 8 - run ctre motionProfile

 //  pidRotateMagic.run(90.0);
    if (joy0.getButton(5) &&!joy0.getPrevButton(5) ) {
        m_autoSelected = m_chooser.getSelected();
        switch (m_autoSelected) {
            case "Rotate90":
                pidRotate.run(90.0,true);
            break;
            case "Rotate180":
                pidRotate.run(90.0,true);
                break;
            case "Straight120_48ips":
                pidStraightMagic.run(120.0,3000,6000);    
            break;                
            case "Straight120_96ips":
                pidStraightMagic.run(120.0,6000,12000);    
            break;               
            case "Profile_straight60_48":
                mpctre.runProfile(straight60_48); 
             break;                           
            case "Profile_straightarc60_48":
                mpctre.runProfile(straightarc60_48); 
             break;                           
            case "Profile_straightarc120_48":
               mpctre.runProfile(straightarc120_48); 
            break;                           
            case "Profile_straightarc120_96":
                mpctre.runProfile(straightarc120_96);    
            break;                           
            case "Profile_WOF_Goal":
                mpctre.runProfile(wof_goal);    
            break;                           

            default:
              // Put default auto code here
              break;
          }

    }


// run lightlight generated profile
    else if (joy0.getButton(7)  && !joy0.getPrevButton(7) && !runningPID){}
   //             limelight.driveStraightToTarget();

 
// else drive manually         
    if(!runningPID) 
        drive.setMotors(joy0.getLeft(),joy0.getRight(),ControlMode.Velocity);
    
     
    
    drive.writeEncoderData();
    limelight.getLimelightData();
    counter++;
//    log.updateTopics();
//    log.log();


  }

}

