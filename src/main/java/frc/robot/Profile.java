//  Profile format:

//  type 1 - single profile using position and velocity of robot center 
//  file format: 
//      numpoints
//      time, vel, pos, angle, action1, action2
//         ...

// type 2 - Create 1 profile stream with velR in arb feed forward and velL in aux arb feed forwars
//  Master controllor is right side. PID polarity must be standard:  primary: PID0 + PID1   Aux: PID0-PID1
//  arbfeedforward = 0.5*(velR + velL)    auxarbfeedforward left = 0.5(velR - velL)
//  position is position of the robot center 
//  file format:    
//      numpoints
//      time, velR, velL, pos, angle, action1, action2
//         ...


package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import java.io.BufferedReader;

public class Profile {
    int size;
    int[] action1,action2;
    BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
    Profile(String name, int type){


        try {
            File filename = new File(name);
            BufferedReader br= new BufferedReader(new FileReader(filename));
            TrajectoryPoint point = new TrajectoryPoint();
            String[] sarray = new String[8];
            double[] time,velLeft,velRight,angle, pos,vel;        
            String s;
            int i = 0;
            size = Integer.parseInt(br.readLine());
            time = new double[size];
            vel=new double[size];
            velLeft=new double[size];
            velRight=new double[size];
            pos=new double[size];
            angle=new double[size];
            action1=new int[size];
            action2=new int[size];


            while (i <size) {
                s="";
                s=br.readLine();
           
                sarray=s.split(",");
                time[i]=Double.parseDouble(sarray[0]);    
                vel[i]=Double.parseDouble(sarray[1]);
                velRight[i]=Double.parseDouble(sarray[2]);
                velLeft[i]=Double.parseDouble(sarray[3]);
                pos[i]=Double.parseDouble(sarray[4]);
                angle[i]=Double.parseDouble(sarray[5]);
                action1[i]=Integer.parseInt(sarray[6]);
                action2[i]=Integer.parseInt(sarray[7]);
           
                point.position =(int) (pos[i] * Constants.kSensorUnitsPerInch);
                if (type==1)   
                    point.velocity = (int)(vel[i] * Constants.k_InchPerSecToVelUnits);
                else {
                    point.velocity = 0.0 * Constants.k_InchPerSecToVelUnits;
                    
                    // aux polarity set to true -> right side gets difference, left side gets sum 
                    point.arbFeedFwd =
                        0.5* (velLeft[i] + velRight[i])*Constants.k_InchPerSecToVelUnits/Constants.maxVelUnitsPer100ms;    
                    point.auxiliaryArbFeedFwd=
                        0.5*(velLeft[i] - velRight[i])*Constants.k_InchPerSecToVelUnits/Constants.maxVelUnitsPer100ms; 
                } 	
                System.out.println("i="+i+"  pos="+point.position+"   arbff="+point.arbFeedFwd);

                point.auxiliaryPos = (int)(angle[i]*8192/360); /* scaled such that 3600 => 360 deg */
                point.profileSlotSelect0 = Constants.slot_pos; // slot for position
                point.profileSlotSelect1 = Constants.slot_angleMP; // slot for angle
                point.timeDur = 10;		
                point.useAuxPID = true;
                point.zeroPos = false;
                if (i == 0){
                    point.zeroPos = true; /* set this to true on the first point */
                }
                point.isLastPoint = false;
                if ((i ) == size-1){
                    point.isLastPoint = true; /* set this to true on the last point  */
                 }
                 stream.Write(point);
            i++;
            }
            br.close();
        } catch (IOException e) {
            System.out.println("Could not load profile "+name);
        }


    }
    
}