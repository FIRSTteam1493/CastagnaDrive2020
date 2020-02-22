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
    static double maxRPM,maxVelUnitsPer100ms,ramptime,turnSF,forwardSF,
            oneWayRampTime,timeOnTargetGoal;

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
        if(Robot.FX){
            kSensorUnitsPerRotation=2048*10.39;  
            wheelDiam = 5.9609;
            wheelCircum = Math.PI*wheelDiam; //
            kSensorUnitsPerInch=(kSensorUnitsPerRotation/wheelCircum);  //  1136.276
            k_InchPerSecToVelUnits=(kSensorUnitsPerInch/10);  //  113.6276
            maxRPM=18000;  //  (actually max vel units/100ms)
//            maxVelUnitsPer100ms=468;  // ,measured on blocks  (encoder units/100ms)
            maxVelUnitsPer100ms=14000;
            // kP  KI,  kD,   kIz,   kFF,   kMin,   kMax,   Key for smartdash;
            vel=new Gains(0.0050, 0.0, 0.0, 20, 0.05, -1, 1, "vel");  // 1023/468
            posMP=new Gains(0.2, 0, 0.7, 0,  0.05, -1, 1, "posMP");
            pos=new Gains(0.034900, 0, 0, 0, 0.0593, -1, 1, "pos");
            angleRot=new Gains(0.0175, 0.0, 0.170, 0, 0.0, -1.0, 1.0, "rotate");
            angleMP=new Gains(0.026, 0, 0, 0, 0, -0.5, 0.5, "angleMP");
            sonar=new Gains(0, 0, 0, 0, 0.0, -1, 1, "sonar");
            other=new Gains(0, 0, 0, 0, 0.0 , -1, 1, "other");    
        }

        else{
            kSensorUnitsPerRotation=512;  // use this for 2019 robot
            wheelDiam = 5.9606;
            wheelCircum = Math.PI*wheelDiam; //17.09
            kSensorUnitsPerInch=(kSensorUnitsPerRotation/wheelCircum);  // 30
            k_InchPerSecToVelUnits=(kSensorUnitsPerInch/10);  // 3
            maxRPM=18000;  //  (actually max vel units/100ms)
            maxVelUnitsPer100ms=468;  // ,measured on blocks  (encoder units/100ms)
            vel=new Gains(0.0005, 0.0, 0.0, 20, 0.05, -1, 1, "vel");  // 1023/468
            posMP=new Gains(3, 0, 1, 0, 2.158, -1, 1, "posMP");
            pos=new Gains(0, 0, 0, 0, 0, -1, 1, "pos");
            angleRot=new Gains(0.0175, 0.0, 0.170, 0, 0.0, -1.0, 1.0, "rotate");
            angleMP=new Gains(3, 0, 2, 0, 0, -0.5, 0.5, "angleMP");
            sonar=new Gains(0, 0, 0, 0, 0.0, -1, 1, "sonar");
            other=new Gains(0, 0, 0, 0, 0.0 , -1, 1, "other");  
        }


// These are independant of drive motor type        
        k_gyroUnitsPerDegree=(8192.0/360.0);
        slot_vel=0;
        slot_pos=1;
        slot_rotate=2;
        slot_angleMP=3;
        timeOnTargetGoal=0.2;
        turnSF=0.5;
        forwardSF=0.7;
        ramptime=0;
        oneWayRampTime=0.015;
    
    
    
    }

    static public void writeGains(){

        posMP.writeGains();
        pos.writeGains();
        vel.writeGains();
        angleRot.writeGains();
        other.writeGains();
        angleMP.writeGains();
        sonar.writeGains();
        SmartDashboard.putNumber("PIDTuning/Max RPM", maxRPM);
        SmartDashboard.putNumber("PIDTuning/TOT goal", timeOnTargetGoal);
     }

    static public void readGains(){
        posMP.readGains();
        pos.readGains();
        vel.readGains();
        angleRot.readGains();
        other.readGains();
        angleMP.readGains();
        sonar.readGains();
        maxRPM = SmartDashboard.getNumber("PIDTuning/Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("PIDTuning/TOT goal",0);
    }

    static public void readangleRotGains(){
        angleRot.readGains();
        maxRPM = SmartDashboard.getNumber("PIDTuning/Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("PIDTuning/TOT goal",0);
    }

    static public void readPosMPGains(){
        posMP.readGains();
        maxRPM = SmartDashboard.getNumber("PIDTuning/Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("PIDTuning/TOT goal",0);
    }


    static public void readPosGains(){
        pos.readGains();
        maxRPM = SmartDashboard.getNumber("PIDTuning/Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("PIDTuning/TOT goal",0);
    }
    
    static public void readOtherGains(){
        other.readGains();
        maxRPM = SmartDashboard.getNumber("PIDTuning/Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("PIDTuning/TOT goal",0);
    }


    static public void readDriveParams(){
        ramptime = SmartDashboard.getNumber("Acc/Dec Ramp",0);
        oneWayRampTime=SmartDashboard.getNumber("Acc Only Ramp",0);
        turnSF = SmartDashboard.getNumber("turnSF",0.5);
        forwardSF = SmartDashboard.getNumber("forwardSF",0.75);
    }

    static public void writeDriveParams(){
        SmartDashboard.putNumber("Acc/Dec Ramp",ramptime);
        SmartDashboard.getNumber("Acc Only Ramp",oneWayRampTime);
        SmartDashboard.getNumber("turnSF",turnSF);
        SmartDashboard.getNumber("forwardSF",forwardSF);
    }

    }


