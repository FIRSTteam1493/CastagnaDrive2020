// to do  - loop for motionprofileCTRE should be in autoperiodic or teleopperiodic
// edit smartdash
// Comment 2/8
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANDigitalInput;
import org.opencv.core.Mat;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
//import badlog.lib.BadLog;

public class Robot extends TimedRobot {
    boolean LEDrelay=false;
    static boolean runningPID = false;
    static boolean autoStarted = false;


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
    LEDdriver led = new LEDdriver();
    Relay relayLimelight = new Relay(3);
    Relay relayLED = new Relay(1);
    Compressor compressor = new Compressor();
    Auto auto = new Auto(drive,joy0,arm,mpctre);
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
    double[] ypr_deg={0,0,0};
    CANDigitalInput reverseLimit;
//    ColorSensor colorSensor = new ColorSensor();
    Mat frame = new Mat();
    int pattern=1;



  @Override
  public void robotInit() {
    SmartDashboard.putString("Messages", "Hello");
    Constants.writeGains();
    Constants.writeDriveParams();
    //camera = new VideoCapture(0);
    CameraServer.getInstance().startAutomaticCapture();

    m_chooser.setDefaultOption("do_nothing", "do_nothing");
    m_chooser.addOption("shoot3_straight", "shoot3_straight");
    m_chooser.addOption("push_shoot3", "push_shoot3");
    m_chooser.addOption("shoot3_side", "shoot3_side");
    m_chooser.addOption("shoot3_side_delay", "shoot3_side_delay");
    m_chooser.addOption("trench_shoot5", "trench_shoot5");
    m_chooser.addOption("backup_pass", "backup_pass");
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

}


  @Override
  public void autonomousInit() {
      m_autoSelected = m_chooser.getSelected();
      drive.resetEncoders();
      drive.resetGyro();
  }


  @Override
  public void autonomousPeriodic() {
    if(!autoStarted) auto.runAuto(m_autoSelected);

  }

  
  @Override
  public void teleopInit() {
    autoStarted=false;
    
  }

  @Override
  public void teleopPeriodic() {

    joy0.readDriverStick();
    joy1.readOperatorStick();

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
        pattern++;if(pattern>7)pattern=0;
        led.sendData(pattern);
        // camera.read(frame);
        // grip.process(frame);
    }

    //read drive paramaters
    else if (joy0.getButton(4) && !joy0.getPrevButton(4) ){
            Constants.readDriveParams();
            drive.setRampTime(Constants.ramptime);
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

    if(joy1.getButton(7)) arm.wofIn();
    else if (joy1.getButton(8)) arm.wofOut();
    else if(!runningPID)  arm.wofStop();

    if (joy1.getButton(9) && !joy1.getPrevButton(9) )  elevator.down();
    else if (joy1.getButton(10) && !joy1.getPrevButton(10) )  elevator.up();        


 
// else drive manually         
    if(!runningPID) 
        drive.setMotors(joy0.getLeft(),joy0.getRight(),ControlMode.Velocity);
    
    if(joy1.isPushed())arm.manualSetPosition(joy1.forward);
     
    arm.brakeMonitor();
    drive.writeEncoderData();
//    limelight.getLimelightData();
//    arm.writeArmData();
    

//    log.updateTopics();
//    log.log();


  }

}

