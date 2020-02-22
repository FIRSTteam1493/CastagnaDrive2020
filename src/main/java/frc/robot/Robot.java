// to do  - loop for motionprofileCTRE should be in autoperiodic or teleopperiodic
// edit smartdash
// Comment 2/8
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANDigitalInput;
import org.opencv.core.Mat;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.opencv.videoio.VideoCapture;
import edu.wpi.first.wpilibj.Relay;
//import badlog.lib.BadLog;

public class Robot extends TimedRobot {
    boolean LEDrelay=false;
    static boolean runningPID = false;
    static boolean turbo = false;
    public static boolean FX = true;
    Constants constants = new Constants();
//    BadLog log;
    VideoCapture camera;
    GripPipeline grip = new GripPipeline();
    Sonar sonar = new Sonar(1);
 //   SonarDigital sonarDigital = new SonarDigital();
    Stick joy0 = new Stick(0), joy1=new Stick(1);
    FalconDriveCTRE drive = new FalconDriveCTRE();  
    Elevator elevator = new Elevator();
    Arm arm = new Arm();
    MotionProfileCTRE mpctre = new MotionProfileCTRE(drive, joy0, arm);
    Limelight limelight = new Limelight(joy0, drive, mpctre);
    PIDRotate pidRotate = new PIDRotate(drive,joy0);  
    PIDRotateMagic pidRotateMagic = new PIDRotateMagic(drive,joy0);  
    PIDStraightMagic pidStraightMagic = new PIDStraightMagic(drive,joy0);
    PIDSonar pidSonar = new PIDSonar(drive,joy0);
    BumpSensor bump = new BumpSensor(drive, joy0);
    LEDdriver led = new LEDdriver();
    Relay relayLimelight = new Relay(3);
    Relay relayLED = new Relay(1);
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
    double[] ypr_deg={0,0,0};
    CANDigitalInput reverseLimit;
//    ColorSensor colorSensor = new ColorSensor();
    Mat frame = new Mat();

    Profile straight60_48,straightarc60_48, straightarc120_48, straightarc120_96, wof_goal,straight83;

  @Override
  public void robotInit() {
    SmartDashboard.putString("Messages", "Hello");
    Constants.writeGains();
    Constants.writeDriveParams();
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
    m_chooser.addOption("Other", "Other");

    SmartDashboard.putData("Auto choices", m_chooser);
    relayLimelight.setDirection(Direction.kForward);
    relayLED.setDirection(Direction.kForward);
    relayLED.set(Value.kOff);
    relayLimelight.set(Value.kOff);
    led.sendData(0); // set LED's red
/*
    log = BadLog.init("/home/lvuser/test.bag");{
    BadLog.createValue("Date", " "+Timer.getFPGATimestamp());
    BadLog.createTopic("Vel_L", "rpm", ()->drive.getlvel()  );
    BadLog.createTopic("Vel_R", "rpm", ()->drive.getrvel()  );
    BadLog.createTopic("Input_L", "units", ()->drive.getLeftInput()  );
    BadLog.createTopic("Input_R", "units", ()->drive.getRightInput()  );
    BadLog.createTopic("CLE0 left", "units", ()->drive.getClosedLoopErrir(0)  );
    BadLog.createTopic("CLE1 right", "units", ()->drive.getClosedLoopErrir(0)  );
        
}

    log.finishInitialization();
*/
    straight60_48 = new Profile("/home/lvuser/profile_straight60_48.profile",2);   
    straightarc60_48 = new Profile("/home/lvuser/profile_straightArc60_48.profile",2);   
    straight83 = new Profile("/home/lvuser/straight83.profile",2);   
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

    joy0.readStick(true);
    joy1.readStick(false);

    if (elevator.calibrated==0 ) {
        System.out.println("Start calibtation0");
        elevator.calibrate();}


// stick 0 actions        

    if(joy0.getButton(1) && !joy0.getPrevButton(1) ){
        Constants.readGains();
        drive.setVelocityGains();
        drive.resetEncoders();
        drive.resetGyro();
        SmartDashboard.putString("Messages", "Gains/Params set");
    }

    else if (joy0.getButton(2)&& !joy0.getPrevButton(2)) {
//  colorSensor.getColor();
        if(LEDrelay){
            relayLED.set(Value.kOff);
            relayLimelight.set(Value.kOff);
            led.sendData(0);
         }
        else {
            relayLED.set(Value.kOn);
            relayLimelight.set(Value.kOn);
            led.sendData(1);
            }
        LEDrelay=!LEDrelay;
    }

    else if (joy0.getButton(3) && !joy0.getPrevButton(3) ){
        // camera.read(frame);
        // grip.process(frame);
    }
    
    //read drive paramaters
    else if (joy0.getButton(4) && !joy0.getPrevButton(4) ){
            Constants.readDriveParams();
            drive.setRampTime(Constants.ramptime);
            }

    else if (joy0.getButton(5) &&!joy0.getPrevButton(5) ) {
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
            case "Other":
            mpctre.runProfile(straight83);    
        break;                           
            default:
              // Put default auto code here
          break;
      }
    }
    if (joy0.getButton(7)  && !joy0.getPrevButton(7) && !runningPID){}
    //             limelight.driveStraightToTarget();

    if (joy0.getButton(8))  turbo=true;
    else turbo=false;
 

// *************************************
// ***    stick 1 actions            ***
// *************************************    
    if(joy1.getButton(1)) arm.setPosition(1);
    else if(joy1.getButton(2)) arm.setPosition(2);
    else if(joy1.getButton(3)) arm.setPosition(3);
    else if(joy1.getButton(4)) arm.setPosition(4);
    
    if(joy1.getButton(5)) arm.shooterIn();
    else if (joy1.getButton(6)) arm.shooterOut();
    else if(!runningPID)  arm.shooterStop();

    if (joy1.getButton(8) && !joy1.getPrevButton(8)) arm.toggleBrake();  

    if (joy1.getButton(9) && !joy1.getPrevButton(9) )  elevator.down();
    else if (joy1.getButton(10) && !joy1.getPrevButton(10) )  elevator.up();        


 
// else drive manually         
    if(!runningPID) 
        drive.setMotors(joy0.getLeft(),joy0.getRight(),ControlMode.Velocity);
//        drive.setMotors(joy0.getLeft(),joy0.getRight(),ControlMode.PercentOutput);
    
    if(joy1.isPushed())arm.manualSetPosition(joy1.forward);
     
    
    drive.writeEncoderData();
    limelight.getLimelightData();
    arm.writeArmData();
    

//    log.updateTopics();
//    log.log();


  }

}

