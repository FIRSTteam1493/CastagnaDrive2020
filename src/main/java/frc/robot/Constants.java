package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// ****************************************************************************
//  The constants class can store up to 6 set of constants for use in PID control.
//  The falcons and spark max controller have on board slots for 
//   up to 4 sets of constants.  Slots are named here and assigned a number.
//  
//  The readGains method reads new values from the smartdashboard
//  The writeGains method writes gains to the smartdashboard 
//      - writing gains needs to be done only once to establish them
//        on the smartdash, since they are changed only by the user via smartdash
//
//  Assignment of gains to motor controllor slots is handled in the drive class
// ******************************************************************************   

public class Constants{

//    static double kSensorUnitsPerRotation=512;  // use this for 2019 robot
    static double kSensorUnitsPerRotation,wheelDiam,wheelCircum;
    static double kSensorUnitsPerInch,k_InchPerSecToVelUnits,k_gyroUnitsPerDegree;
    static int slot_vel,slot_pos, slot_rotate, slot_angleMP;
    static double maxVelUnitsPer100ms,timeOnTargetGoal,errorAllowable,minTurnInp,
         ramptime,deltaXMax,turnSF,forwardSF, oneWayRampTime;

// Inner class to hold groups of PIDF gains 
// kP  KI,  kD,   kIz,   kFF,   kMin,   kMax,   Key for smartdash;

// Add kF for magic motion 
   static Gains vel, posMP, pos, angleRot, angleMP, sonar , other;

    static class Gains{
        double kP,kI,kD,kIz,kFF,kMin,kMax;
        String key;
        Gains(double _kP,double _kI,double _kD,double _kIz,double _kFF,
        double _kMin,double _kMax,String _key){
            kP=_kP;kI=_kI;kD=_kD;kIz=_kIz;kFF=_kFF;kMin=_kMin;kMax=_kMax;key=_key;
        }

        private void writeGains(){
            SmartDashboard.putNumber(("PIDTuning/kP "+key), this.kP);
            SmartDashboard.putNumber(("PIDTuning/kI "+key), this.kI);
            SmartDashboard.putNumber(("PIDTuning/kD "+key), this.kD);
            SmartDashboard.putNumber(("PIDTuning/kIz "+key), this.kIz);
            SmartDashboard.putNumber(("PIDTuning/kFF "+key), this.kFF);
            SmartDashboard.putNumber(("PIDTuning/MaxOut "+key), this.kMax);
            SmartDashboard.putNumber(("PIDTuning/MinOut "+key), this.kMin);
        }

        private void readGains(){
            this.kP=SmartDashboard.getNumber(("PIDTuning/kP "+key),0);
            this.kI=SmartDashboard.getNumber(("PIDTuning/kI "+key),0);
            this.kIz = SmartDashboard.getNumber(("PIDTuning/kIz "+key),0);
            this.kD = SmartDashboard.getNumber(("PIDTuning/kD "+key),0); 
            this.kFF = SmartDashboard.getNumber(("PIDTuning/kFF "+key),0); 
            this.kMin= SmartDashboard.getNumber(("PIDTuning/MinOut "+key),0); 
            this.kMax= SmartDashboard.getNumber(("PIDTuning/MaxOut "+key),0);
        }
    }           
    
    
    Constants(){
    
        kSensorUnitsPerRotation=2048*10.39;  
        wheelDiam = 5.9609;
        wheelCircum = Math.PI*wheelDiam; //
        kSensorUnitsPerInch=(kSensorUnitsPerRotation/wheelCircum);  //  1136.276
        k_InchPerSecToVelUnits=(kSensorUnitsPerInch/10);  //  113.6276
        maxVelUnitsPer100ms=14000;
        // kP  KI,  kD,   kIz,   kFF,   kMin,   kMax,   Key for smartdash;
        vel=new Gains(0.0050, 0.0, 0.0, 20, 0.05, -1, 1, "vel");  // 1023/468
        posMP=new Gains(0.15, 0, 0.8, 0,  0.05, -1, 1, "posMP");
        pos=new Gains(0.034900, 0, 0, 0, 0.0593, -1, 1, "pos");
        angleRot=new Gains(0.0175, 0.0, 0.170, 0, 0.0, -0.5, 0.5, "rotate");
        angleMP=new Gains(0.35, 0, .1, 0, 0, -0.5, 0.5, "angleMP");
        sonar=new Gains(0, 0, 0, 0, 0.0, -1, 1, "sonar");
        other=new Gains(0, 0, 0, 0, 0.0 , -1, 1, "other");    

// These are independant of drive motor type        
        k_gyroUnitsPerDegree=(8192.0/360.0);
        slot_vel=0;
        slot_pos=1;
        slot_rotate=2;
        slot_angleMP=3;
        timeOnTargetGoal=0.2;
        errorAllowable=0.5;
        minTurnInp=0.04;
        turnSF=0.8;
        forwardSF=0.8;
        ramptime=0.5;
        oneWayRampTime=3.0;
    }

    static public void writeGains(){

        posMP.writeGains();
        pos.writeGains();
        vel.writeGains();
        angleRot.writeGains();
        other.writeGains();
        angleMP.writeGains();
        sonar.writeGains();
        SmartDashboard.putNumber("PIDTuning/Max Vunits", maxVelUnitsPer100ms);
        SmartDashboard.putNumber("PIDTuning/TOT goal", timeOnTargetGoal);
        SmartDashboard.putNumber("PIDTuning/Error Allow", errorAllowable);
        SmartDashboard.putNumber("PIDTuning/Min Turn Inp", minTurnInp);
     }

    static public void readGains(){
        posMP.readGains();
        pos.readGains();
        vel.readGains();
        angleRot.readGains();
        other.readGains();
        angleMP.readGains();
        sonar.readGains();
        readPIDParams();
    }

    static public void readangleRotGains(){
        angleRot.readGains();
        readPIDParams();
    }

    static public void readPosMPGains(){
        posMP.readGains();
        readPIDParams();
    }


    static public void readPosGains(){
        pos.readGains();
        readPIDParams();
    }
    
    static public void readOtherGains(){
        other.readGains();
        readPIDParams();
    }


    static public void readDriveParams(){
        ramptime = SmartDashboard.getNumber("Drive/Acc Dec Ramp",0);
        oneWayRampTime=SmartDashboard.getNumber("Drive/Acc Only Ramp",0);
        turnSF = SmartDashboard.getNumber("Drive/turnSF",0.5);
        forwardSF = SmartDashboard.getNumber("Drive/forwardSF",0.75);
    }

    static public void readPIDParams(){
        maxVelUnitsPer100ms = SmartDashboard.getNumber("PIDTuning/Max Vunits",0);
        timeOnTargetGoal = SmartDashboard.getNumber("PIDTuning/TOT goal",0);
        errorAllowable = SmartDashboard.getNumber("PIDTuning/Error Allow",0);
        minTurnInp=SmartDashboard.getNumber("PIDTuning/Min Turn Inp", 0);
    }


    static public void writeDriveParams(){
        SmartDashboard.putNumber("Drive/Acc Dec Ramp",ramptime);
        SmartDashboard.putNumber("Drive/Acc Only Ramp",oneWayRampTime);
        SmartDashboard.putNumber("Drive/turnSF",turnSF);
        SmartDashboard.putNumber("Drive/forwardSF",forwardSF);
    }

    }


