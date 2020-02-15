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
    static double maxRPM,maxVelUnitsPer100ms,ramptime,timeOnTargetGoal;

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
            SmartDashboard.putNumber("ramp time",ramptime);
            SmartDashboard.putNumber(("kP "+key), this.kP);
            SmartDashboard.putNumber(("kI "+key), this.kI);
            SmartDashboard.putNumber(("kD "+key), this.kD);
            SmartDashboard.putNumber(("kIz "+key), this.kIz);
            SmartDashboard.putNumber(("kFF "+key), this.kFF);
            SmartDashboard.putNumber(("MaxOut "+key), this.kMax);
            SmartDashboard.putNumber(("MinOut "+key), this.kMin);
        }

        private void readGains(){
            this.kP=SmartDashboard.getNumber(("kP "+key),0);
            this.kI=SmartDashboard.getNumber(("kI "+key),0);
            this.kIz = SmartDashboard.getNumber(("kIz "+key),0);
            this.kD = SmartDashboard.getNumber(("kD "+key),0); 
            this.kFF = SmartDashboard.getNumber(("kFF "+key),0); 
            this.kMin= SmartDashboard.getNumber(("MinOut "+key),0); 
            this.kMax= SmartDashboard.getNumber(("MaxOut "+key),0);
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
            maxVelUnitsPer100ms=18000;
            // kP  KI,  kD,   kIz,   kFF,   kMin,   kMax,   Key for smartdash;
            vel=new Gains(0.000050, 0.0, 0.0, 20, 0.0593, -1, 1, "vel");  // 1023/468
            posMP=new Gains(0.034900, 0, 1, 0,  0.0593, -1, 1, "posMP");
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
            vel=new Gains(0.00005, 0.0, 0.0, 20, 0.05683, -1, 1, "vel");  // 1023/468
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
        ramptime=0;
        timeOnTargetGoal=0.2;
    
    
    
    }

    static public void writeGains(){

        posMP.writeGains();
        pos.writeGains();
        vel.writeGains();
        angleRot.writeGains();
        other.writeGains();
        angleMP.writeGains();
        sonar.writeGains();
        SmartDashboard.putNumber("Max RPM", maxRPM);
        SmartDashboard.putNumber("TOT goal", timeOnTargetGoal);
        SmartDashboard.putNumber("ramp time", ramptime);
     }

    static public void readGains(){
        posMP.readGains();
        pos.readGains();
        vel.readGains();
        angleRot.readGains();
        other.readGains();
        angleMP.readGains();
        sonar.readGains();
        maxRPM = SmartDashboard.getNumber("Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("TOT goal",0);
        ramptime = SmartDashboard.getNumber("ramptime",0);
    }

    static public void readangleRotGains(){
        angleRot.readGains();
        maxRPM = SmartDashboard.getNumber("Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("TOT goal",0);
        ramptime = SmartDashboard.getNumber("ramptime",0);
    }

    static public void readPosMPGains(){
        posMP.readGains();
        maxRPM = SmartDashboard.getNumber("Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("TOT goal",0);
        ramptime = SmartDashboard.getNumber("ramptime",0);
    }


    static public void readPosGains(){
        pos.readGains();
        maxRPM = SmartDashboard.getNumber("Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("TOT goal",0);
        ramptime = SmartDashboard.getNumber("ramptime",0);
    }
    
    static public void readOtherGains(){
        other.readGains();
        maxRPM = SmartDashboard.getNumber("Max RPM",0);
        timeOnTargetGoal = SmartDashboard.getNumber("TOT goal",0);
        ramptime = SmartDashboard.getNumber("ramptime",0);
    }
    }


