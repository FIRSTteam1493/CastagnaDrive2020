
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconDriveCTRE{
    
    TalonFX bl = new TalonFX(3);
    TalonFX br = new TalonFX(1);
    TalonFX fl = new TalonFX(4);
    TalonFX fr = new TalonFX(2);

    double leftIn=0,rightIn=0;
    FeedbackDevice fbd;
    PigeonIMU gyro;

    FalconDriveCTRE(){
        fbd=FeedbackDevice.IntegratedSensor;
        gyro= new PigeonIMU(10);
       initializeTalon();

     }

public void setRampTime(double ramptime){
// set ramp time in seconds
    bl.configOpenloopRamp(Constants.ramptime);
    fl.configOpenloopRamp(Constants.ramptime);
    br.configOpenloopRamp(Constants.ramptime);
    fr.configOpenloopRamp(Constants.ramptime);
}

// set motor mode to brake or coast
public void setBrakeMode(){
    bl.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    fl.setNeutralMode(NeutralMode.Brake);
    fr.setNeutralMode(NeutralMode.Brake);
}

public void setMotors(double leftinput, double rightinput, ControlMode mode){
    // velocity setpoint is in units/100ms
    double conversion=1.0;
    if (mode==ControlMode.Velocity) 
        conversion=Constants.maxVelUnitsPer100ms;
    if(mode==ControlMode.Position)conversion= Constants.kSensorUnitsPerRotation*4; // convert to sensor_frunningunits
    leftIn=leftinput*conversion;
    rightIn=rightinput*conversion;
    bl.set(mode,leftIn);
    br.set(mode,rightIn);
 }

 public void setMotors(double leftinput, double rightinput){
    leftIn=leftinput;
    rightIn=rightinput;
    bl.set(ControlMode.PercentOutput,leftinput);
    br.set(ControlMode.PercentOutput,rightinput);
}


public void setMotorsPos(double input){
    double targetPos=input*512*4;
    br.set(ControlMode.Position, targetPos);
    bl.follow(br);
}



public double getlpos(){
    return bl.getSelectedSensorPosition();
} 
public double getrpos(){
    return br.getSelectedSensorPosition();
}
public double getlvel(){
    return bl.getSelectedSensorVelocity();
} 
public double getrvel(){
    return br.getSelectedSensorVelocity();
}  
public double getLeftInput(){
    return leftIn;
}  
public double getRightInput(){
    return rightIn;
}  
public double getAngle(){
    double[] ypr_deg=new double[3];
    gyro.getYawPitchRoll(ypr_deg);
    return ypr_deg[0];
}  

public void getCurrent(){
    SmartDashboard.putNumber("Drive/FL Current", fl.getStatorCurrent());
    SmartDashboard.putNumber("Drive/BL Current", bl.getStatorCurrent());
    SmartDashboard.putNumber("Drive/FR Current", fr.getStatorCurrent());
    SmartDashboard.putNumber("Drive/BR Current", br.getStatorCurrent());
}  


public void resetSensors(){
    resetEncoders();
    resetGyro();
} 

public void resetEncoders(){
    bl.setSelectedSensorPosition(0);
    br.setSelectedSensorPosition(0);
} 

public void resetGyro(){
    gyro.setYaw(0);
} 


public void writeEncoderData(){
    SmartDashboard.putNumber("Drive/Left Pos", getlpos());
    SmartDashboard.putNumber("Drive/Right Pos", getrpos());
 //   SmartDashboard.putNumber("Drive/Left Vel", getlvel());
 //   SmartDashboard.putNumber("Drive/Right Vel", getrvel() );
    SmartDashboard.putNumber("Drive/Left Vel RPM", getlvel()*600/Constants.kSensorUnitsPerRotation);
    SmartDashboard.putNumber("Drive/Right Vel RPM", getrvel()*600/Constants.kSensorUnitsPerRotation );
    SmartDashboard.putNumber("Drive/Angle", getAngle() );
//    SmartDashboard.putNumber("PIDTuning/CLE0_Right", br.getClosedLoopError(0));
//    SmartDashboard.putNumber("PIDTuning/CLE1_Right", br.getClosedLoopError(1));
    SmartDashboard.putNumber("Drive/FL Pos", fl.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Drive/FR Pos", fr.getSelectedSensorPosition(0));

  //  System.out.println(br.getClosedLoopError(0) + "  " + br.getClosedLoopTarget() + "   " + br.getSelectedSensorVelocity());

}


// Set PID Gains       
public void setPIDGains(int slot, Constants.Gains gains){
    bl.config_kP(slot, gains.kP );
    bl.config_kI(slot, gains.kI);
    bl.config_kD(slot, gains.kD);
    bl.config_IntegralZone(slot, (int)gains.kIz);
    bl.config_kF(slot, gains.kFF);
    bl.configClosedLoopPeakOutput(slot,gains.kMax);
    br.config_kP(slot, gains.kP );
    br.config_kI(slot, gains.kI);
    br.config_kD(slot, gains.kD);
    br.config_IntegralZone(slot, (int)gains.kIz);
    br.config_kF(slot, gains.kFF);
    br.configClosedLoopPeakOutput(slot,gains.kMax);    
   }

   public void setVelocityGains(){
       setPIDGains(Constants.slot_vel,Constants.vel);
   }

public void setPositionGains(){
    setPIDGains(Constants.slot_pos,Constants.pos);
}
public void setRotateGains(){
    setPIDGains(Constants.slot_rotate,Constants.angleRot);
}



//*************************************************
//*    Talon Initial Setup                        *
//*************************************************    
public void initializeTalon(){
br.configFactoryDefault();
fr.configFactoryDefault();
bl.configFactoryDefault();
fl.configFactoryDefault();  
br.configVoltageCompSaturation(12);
bl.configVoltageCompSaturation(12);
fr.configVoltageCompSaturation(12);
bl.configVoltageCompSaturation(12);
bl.configNeutralDeadband(0.005);
br.configNeutralDeadband(0.005);
fl.configNeutralDeadband(0.005);
fr.configNeutralDeadband(0.005);
setBrakeMode();    

// invert the right side
br.setInverted(true);
fr.setInverted(true);
bl.setInverted(false);
fl.setInverted(false);

bl.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
br.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
fl.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
fr.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
// set the sensor phase 
//bl.setSensorPhase(true);
//br.setSensorPhase(false);

// front follows back
fr.follow(br);
fl.follow(bl);

// set closed loop period    
int closedLoopTimeMs = 1;
bl.configClosedLoopPeriod(0, closedLoopTimeMs,20);
br.configClosedLoopPeriod(0, closedLoopTimeMs,20);

// Set relevant frame periods to be at least as fast as periodic rate */

br.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 20);
br.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 20);
br.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20,20);
br.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20,20);
br.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 20);               
gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, 20);
    

// load all the gains
setPIDGains(Constants.slot_pos,Constants.posMP);    
setPIDGains(Constants.slot_vel,Constants.vel);    
setPIDGains(Constants.slot_rotate,Constants.angleRot);    
setPIDGains(Constants.slot_angleMP,Constants.other);    

// use the velocity PID gains for the primary PID loop     
br.selectProfileSlot(Constants.slot_vel, 0);
bl.selectProfileSlot(Constants.slot_vel, 0);

setRampTime((Constants.ramptime));
resetSensors();
}
   



//*************************************************
//*    Set Talon Parameters for MP Operation  *
//*************************************************    
   public void setupTalonMP(){
    setupTalonMotionMagicStraight();
    setRampTime((0.0));
   }


//*************************************************************************
//*    Set Talon Parameters for Rotate  using MotionMagic                 * 
//*     right talon is the master, note - position PID required FF term!  *
//*************************************************************************    
public void setupTalonMotionMagicRotate(){
    bl.setInverted(true);
    fl.setInverted(true);
    br.setInverted(true);
    fr.setInverted(true);

    //Set talon sensors for arcade style motion profile    
    // right master,   PID0 = gyro  PID1 = senor difference        

    bl.configSelectedFeedbackSensor(fbd, 0,20);       //sensor, pid#
    
    // remote sensor 0 is the gyro         
    br.configRemoteFeedbackFilter(gyro.getDeviceID(),
        RemoteSensorSource.GadgeteerPigeon_Yaw, 0,20);  

    // remote sensor 1 is the left side encoder    
    br.configRemoteFeedbackFilter(bl.getDeviceID(),
            RemoteSensorSource.TalonFX_SelectedSensor,1,20);


	// Use the pigeon in remotesensor0 for main pid 0
	br.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);


    // Aux PID 1 uses the difference of the encoder values 
    br.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor1, 20);
	br.configSensorTerm(SensorTerm.Diff1, fbd, 20);
	br.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference,1, 20);

    // scale the gyro term
    br.configSelectedFeedbackCoefficient(	1, 0, 20);


    // set left output = PID0+PID1, left output PID0-PID1   
    br.configAuxPIDPolarity(false, 20);
    
    br.configMotionAcceleration(700, 20);
    br.configMotionCruiseVelocity(350, 20);
    br.configMotionSCurveStrength(4, 20);

    setPIDGains(Constants.slot_rotate,Constants.angleRot);    
    setPIDGains(Constants.slot_pos,Constants.pos);    
   
    br.selectProfileSlot(Constants.slot_rotate, 0);
    br.selectProfileSlot(Constants.slot_pos, 1);
    resetSensors();
   }



//*************************************************************************
//*    Set Talon Parameters for Straight using MotionMagic + Aux Gyro     *
//*      and Motion Profile                                               *
//*     right talon is master                                             *
//*************************************************************************    
public void setupTalonMotionMagicStraight(){
    setRampTime((0.0));
    br.setInverted(true);
    fr.setInverted(true);
    bl.setInverted(false);
    fl.setInverted(false);
    
    //Set talon sensors for arcade style motion profile    
    //sensor, pid#        
    bl.configSelectedFeedbackSensor(fbd, 0,20);
    
    // remote sensor 0 is the left side encoder    
    br.configRemoteFeedbackFilter(bl.getDeviceID(),
            RemoteSensorSource.TalonFX_SelectedSensor,0,20);

    // remote sensor 1 is the gyro         
    br.configRemoteFeedbackFilter(gyro.getDeviceID(),
        RemoteSensorSource.Pigeon_Yaw, 1,20);  
        

    // primary PID 0 uses the sum of the encoder values, but right (master) in inverted
    // use difference:   r - l, but right is inverted so its really -r - l
    // since right is inverted, the difference is inverted and we get r+l
    br.configSensorTerm(SensorTerm.Diff0, fbd, 20);
    br.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, 20);
    br.configSelectedFeedbackSensor(TalonFXFeedbackDevice.SensorDifference,0, 20);

    // multiply the sensor sun by 1/2 
    br.configSelectedFeedbackCoefficient(0.5, 0, 20);

	// Use the pigeon in remotesensor1 for aux pid 1%
	br.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, 0);
    br.configSelectedFeedbackCoefficient(1.0, 1, 20);
    // set left output = PID0+PID1, right output PID0-PID1   
    br.configAuxPIDPolarity(false, 20);
    
    br.configMotionAcceleration(12000, 20);
    br.configMotionCruiseVelocity(6000, 20);
    br.configMotionSCurveStrength(4, 20);

    setPIDGains(Constants.slot_pos,Constants.posMP);
    setPIDGains(Constants.slot_angleMP,Constants.angleMP);    

// use the position gains for primary PID 0 and 'other' gains for aux PID 1     
// feed forward on position, but not on angle 
    br.selectProfileSlot(Constants.slot_pos, 0);
    br.selectProfileSlot(Constants.slot_angleMP, 1);
    resetSensors();
    
    
   }



//******************************************************************
//*    Set Talon Parameters for Teleop Operation                   *
//*     reset parameters that might have been changed in PID mode  *
//******************************************************************   
   public void setupTalonTeleop(){
    setRampTime((Constants.ramptime));
     // use integrated sensor for primary PID 0  
    br.configSelectedFeedbackSensor(fbd, 0, 20);
    bl.configSelectedFeedbackSensor(fbd, 0, 20);

    // reset sensor coeff to 1
    bl.configSelectedFeedbackCoefficient(1.0, 0, 20);
    br.configSelectedFeedbackCoefficient(1.0, 0, 20);

    br.setInverted(true);
    fr.setInverted(true);
    bl.setInverted(false);
    fl.setInverted(false);
    
// use the velocity PID gains for the primary PID loop     
    setPIDGains(Constants.slot_vel,Constants.vel);    
    br.selectProfileSlot(Constants.slot_vel, 0);
    bl.selectProfileSlot(Constants.slot_vel, 0);
}



//******************************************************************
//*    Set Talon Parameters for Bump Sensor                        *
//*     reset parameters that might have been changed in PID mode  *
//******************************************************************   
public void setupTalonBump(){
    setRampTime((0.5));
    setupTalonTeleop();
    br.configRemoteFeedbackFilter(gyro.getDeviceID(),
    RemoteSensorSource.Pigeon_Yaw, 1,20);  
    br.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, 0);
    br.configSelectedFeedbackCoefficient(1.0, 1, 20);
    // set left output = PID0+PID1, right output PID0-PID1   
    br.configAuxPIDPolarity(true, 20);

    setPIDGains(Constants.slot_vel,Constants.vel);    
    setPIDGains(Constants.slot_angleMP,Constants.angleMP);
    br.selectProfileSlot(Constants.slot_vel, 0);
    br.selectProfileSlot(Constants.slot_angleMP, 1);
    br.setSelectedSensorPosition(0);
    resetSensors();
}

}