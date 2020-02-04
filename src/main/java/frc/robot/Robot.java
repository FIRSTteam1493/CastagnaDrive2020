// to do  - loop for motionprofileCTRE should be in autoperiodic or teleopperiodic
// edit smartdash
// Added comment 8:20
// Added Comment 8:27
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANDigitalInput;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import badlog.lib.BadLog;

public class Robot extends TimedRobot {
    static boolean runningPID = false;
    public static boolean FX = true;
    Constants constants = new Constants();
    BadLog log;
    VideoCapture camera;
    GripPipeline grip = new GripPipeline();
    Sonar sonar = new Sonar(1);
 //   SonarDigital sonarDigital = new SonarDigital();
    Stick joy0 = new Stick(0);
    FalconDriveCTRE drive = new FalconDriveCTRE();  
    MotionProfileCTRE mpctre = new MotionProfileCTRE(drive, joy0);
    Limelight limelight = new Limelight(joy0, drive, mpctre);
    Profile arcprofile,straightprofile,rotateprofile;
    PIDRotate pidRotate = new PIDRotate(drive,joy0);  
    PIDRotateMagic pidRotateMagic = new PIDRotateMagic(drive,joy0);  
    PIDStraightMagic pidStraightMagic = new PIDStraightMagic(drive,joy0);
    PIDSonar pidSonar = new PIDSonar(drive,joy0);


    double[] ypr_deg={0,0,0};
    CANDigitalInput reverseLimit;
//    ColorSensor colorSensor = new ColorSensor();
    Mat frame = new Mat();
    int counter=0;


  @Override
  public void robotInit() {
    SmartDashboard.putString("Messages", "Hello");
    Constants.writeGains();
    camera = new VideoCapture(0);

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
   straightprofile = new Profile("/home/lvuser/profile_straight120_48.profile",2);
   arcprofile = new Profile("/home/lvuser/profile_arc120_48.profile",2);   
//   arcprofile = new Profile("/home/lvuser/arc1_24Profile.profile",1);
//   rotateprofile = new Profile("/home/lvuser/rotate1",1);

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
  public void teleopPeriodic() {

    joy0.readStick();
    
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


     // Choose a drive actions  - either run a PID or drive from stick   
    // Button 5 - run Rotate PID
    // Button 6 - run Straight PID
    // Button 7 - position control 
    // Button 8 - run ctre motionProfile

 //  pidRotateMagic.run(90.0);
    if (joy0.getButton(5) &&!joy0.getPrevButton(5) ) 
        pidRotate.run(90.0,true);

// Magic Motion Straight
     else if (joy0.getButton(6) && !joy0.getPrevButton(6)  ) 
         pidStraightMagic.run(60.0);    

// run lightlight generated profile
    else if (joy0.getButton(7)  && !joy0.getPrevButton(7) && !runningPID)
                limelight.driveStraightToTarget();

// run storied profile
    else if (joy0.getButton(8) && !joy0.getPrevButton(8) && !runningPID)
         mpctre.runProfile(straightprofile.stream);
   
// else drive manually         
    if(!runningPID) 
        drive.setMotors(joy0.getLeft(),joy0.getRight(),ControlMode.Velocity);

    
     
    
    drive.writeEncoderData();
    limelight.getLimelightData();
    counter++;
    log.updateTopics();
    log.log();


  }

}

