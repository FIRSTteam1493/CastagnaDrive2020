package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;



public class PathPlanner {
	double halfWheelbase = 23.0/2;
	static List<Double> px = new ArrayList<Double>();
	static List<Double> py = new ArrayList<Double>();
	static List<Double> pxL = new ArrayList<Double>();
	static List<Double> pyL = new ArrayList<Double>();
	static List<Double> pxR = new ArrayList<Double>();
	static List<Double> pyR = new ArrayList<Double>();
	List<Integer> pdir = new ArrayList<Integer>();
	static List<Double> angle1 = new ArrayList<Double>();
	static List<Double> dist1L = new ArrayList<Double>();
	static List<Double> dist1R = new ArrayList<Double>();
	static List<Double> dist1 = new ArrayList<Double>();
	static List<Double> pos1 = new ArrayList<Double>();
	static List<Double> pos1L = new ArrayList<Double>();
	static List<Double> pos1R = new ArrayList<Double>();
	
	static List<Double> time = new ArrayList<Double>();
	static List<Double> pos2 = new ArrayList<Double>();
	static List<Double> pos2L = new ArrayList<Double>();
	static List<Double> pos2R = new ArrayList<Double>();
	static List<Double> dist2L = new ArrayList<Double>();
	static List<Double> dist2R = new ArrayList<Double>();
	static List<Double> dist2 = new ArrayList<Double>();
	static List<Double> vel = new ArrayList<Double>();
	static List<Double> angle2 = new ArrayList<Double>();
	static List<Double> velL = new ArrayList<Double>();
	static List<Double> velR = new ArrayList<Double>();

	final double EPS = 0.000001;
	double	pointsInProfile=0;
	long startTime,endTime;
	double pathlength=0;
    BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
    

	PathPlanner(){
	}
	// comment
	public void makePath(double xS, double yS, double angleS, double xE, double yE, double angleE,
			double vm, double am, double jm) throws IOException {
			startTime = System.nanoTime();
			
			clearProfile();
			double x0,y0,x1,y1,xc0,yc0,xc1,yc1;
			double x,y,xp=0,yp=0,dx,dy,dt,t=0,angle,_angle;
			double xl,yl,xr,yr;
			double distl=0,distr=0,dist=0;
			double pos=0,posl=0,posr=0;
			double temp;
			double ds=0.005;
			dt=0.0001;
			
			angleS=angleS*Math.PI/180.;
			angleE=angleE*Math.PI/180.;
			_angle= angleS;
			
	
			x0=xS;
			y0=yS;
			xc0=x0+20*Math.cos(angleS);
			yc0=y0+20*Math.sin(angleS);
			x1=xE;
			y1=yE;			
			xc1=x1+20*Math.cos(Math.PI+angleE);
			yc1=y1+20*Math.sin(Math.PI+angleE);


			while (t<1.0) {
				double omt = 1.0-t;
				x=omt*omt*omt*x0 + 3*t*omt*omt*xc0 + 3*t*t*omt*xc1 + t*t*t*x1;
				y=omt*omt*omt*y0 + 3*t*omt*omt*yc0 + 3*t*t*omt*yc1 + t*t*t*y1;
				dx=x-xp;
				dy=y-yp;
				pathlength=pathlength+Math.sqrt(dx*dx+dy*dy);				
	
				angle = Math.atan(dy/(dx+EPS));				
				if(t==0) {angle=_angle;	}
				angle1.add( angle);
				xl=x+(halfWheelbase)*Math.cos(angle+Math.PI/2);
				yl=y+(halfWheelbase)*Math.sin(angle+Math.PI/2);
					
				xr=x+(halfWheelbase)*Math.cos(angle+-Math.PI/2);
				yr=y+(halfWheelbase)*Math.sin(angle+-Math.PI/2);
	
				dist=dist+Math.sqrt(dx*dx+dy*dy);
				pos=pos+Math.sqrt(dx*dx+dy*dy);
				double dangle=0;
				
				if(t!=0) dangle=angle1.get(angle1.size()-1) - angle1.get(angle1.size()-2);
				
				temp = Math.sqrt(dx*dx+dy*dy);
				distl=distl+  Math.abs(temp-dangle*halfWheelbase);
				distr=distr+ Math.abs(temp+dangle*halfWheelbase);
				
				posl=posl+ (temp-dangle*halfWheelbase);
				posr=posr+(temp+(dangle)*halfWheelbase);
				py.add(y);
				px.add(x);
					
				pxL.add(xl);
				pyL.add(yl);
					
				pxR.add(xr);
				pyR.add(yr);
					
				xp=x;yp=y;
				_angle=angle;				
					
				dist1.add(dist);			
				dist1L.add(distl);			
				dist1R.add(distr);			
				pos1.add(pos);			
				pos1L.add(posl);			
				pos1R.add(posr);			
				pointsInProfile++;			
				dt=ds/Math.sqrt( (dx/dt)*(dx/dt)+(dy/dt)*(dy/dt));
				if(t==0) dt=0.0001;
				t=t+dt;
				}		
			
			
			calculateVDAprofile(jm,am,vm,0,0,pathlength);			
			saveProfile();
            endTime= System.nanoTime();
			System.out.println("Calc Time= "+(endTime-startTime)/1e6);

	}
	
	
	public void calculateVDAprofile(double jmax, double amax, 
			double vmax, double vi, double vf, double d){
		int i=0,index=0,numSpeedPoints=0;
		double t1,t2,t3,t5,t6,t7;
		double dt=0.0001,totalTime=0;
		double _vel,_dist,_time,angle,_dist2L,_dist2R,_pos,_posl,_posr;
		double dpos2L,dpos2R,vl,vr;
		double timePrevious=0,posPrevious=0,distPrevious=0,temp;

		//vi=0;//vf=0;
//  limit amax to the greatest acceleration achievable given vmax and jmax    
//  i.e., v and j are fixed, tweak a if there are issues     	    	
    	amax= Math.min(Math.sqrt((Math.abs(vmax-vi))*jmax),amax);

// Get the starting sign of acc and jerk correct - start negative if vmax-vi is negative    	
    	amax=Math.abs(amax)*Math.signum(vmax-vi);		
		jmax=Math.abs(jmax)*Math.signum(vmax-vi);
    	
		t1=amax/jmax; 
		if (vmax==vi) {t1=0;}
		double a1=amax;	
		double v1=vi + jmax*t1*t1/2; double s1=vi*t1+ jmax*t1*t1*t1/6;
		
		t3=t1; double a3=0;
		double v3=vmax;
		double a2=amax;
		double v2=v3-a2*t3+jmax*t3*t3/2;
		t2=(v2-v1)/a2;
		double s2=s1+v1*t2+a1*t2*t2/2;
		double s3=s2+v2*t3+a2*t3*t3/2-jmax*t3*t3*t3/6;
		
		double a4=0;double v4=vmax;
		double a5=-amax;
		if (vmax==vf) {t5=0;}else t5=(a5-a4)/(-jmax);  
		double v5=v4-jmax*t5*t5/2;
		double ds5=v4*t5-jmax*t5*t5*t5/6;
		double a6=-amax;
		double a7=0;
		double v7=vf;
		if (vmax==vf) {t7=0;}else t7=amax/jmax;  
		double v6=v7-a6*t7-jmax*t7*t7/2;
		if (amax==0) {t6=0;} else t6=(v6-v5)/(-amax);		
		double ds6=v5*t6+a5*t6*t6/2;
		double ds7 = v6*t7+a6*t7*t7/2+jmax*t7*t7*t7/6;
		double ds4=Math.signum(vmax)*(d-Math.abs(s3+ds5+ds6+ds7));
		double s4=ds4+s3;
		double t4=ds4/v4;
		double s5=s4+ds5;
		double s6=s5+ds6;
		double s7=s6+ds7;
		

		double s=0;
		if (t7>100.) t7=100.;
		double t=0;double ts=0;
		totalTime=t1+t2+t3+t4+t5+t6+t7;
		while (t<totalTime) {
			_time=(t+timePrevious);
			if(t<=t1) {
				ts=t;
				_pos=(posPrevious+vi*t+jmax*ts*ts*ts/6);
				_dist=(distPrevious+Math.abs(vi*t+jmax*ts*ts*ts/6));
				_vel=(vi+jmax*ts*ts/2);
			}			
			else if(t<=t1+t2) {
				ts=t-t1;
				_pos=(posPrevious+s1+v1*ts+amax*ts*ts/2);
				_dist=(distPrevious+Math.abs(s1)+Math.abs(v1*ts+amax*ts*ts/2));
				_vel=(v1+amax*ts);
			}
			else if(t<=t1+t2+t3) {
				ts=t-t1-t2;
				_pos=(posPrevious+s2+v2*ts+amax*ts*ts/2-jmax*ts*ts*ts/6);
				_dist=(distPrevious+Math.abs(s2)+Math.abs(v2*ts+amax*ts*ts/2-jmax*ts*ts*ts/6));
				_vel=(v2+amax*ts-jmax*ts*ts/2);

			}
			else if(t<=t1+t2+t3+t4) {
				ts=t-t1-t2-t3;
				_pos=(posPrevious+s3+vmax*ts);
				_dist=(distPrevious+Math.abs(s3)+Math.abs(vmax*ts));
				_vel=(vmax);
			}
			else if(t<=t1+t2+t3+t4+t5) {
				ts=t-t1-t2-t3-t4;
				_pos=(posPrevious+s4+v4*ts-jmax*ts*ts*ts/6);
				_dist=(distPrevious+Math.abs(s4)+Math.abs(v4*ts-jmax*ts*ts*ts/6));
				_vel=(v4-jmax*ts*ts/2);
			}
			else if(t<=t1+t2+t3+t4+t5+t6) {
				ts=t-t1-t2-t3-t4-t5;
				_pos=(posPrevious+s5+v5*ts-amax*ts*ts/2);
				_dist=(distPrevious+Math.abs(s5)+Math.abs(v5*ts-amax*ts*ts/2));
				_vel=(v5-amax*ts);
			}
			else  {
				ts=t-t1-t2-t3-t4-t5-t6;
				_pos=(posPrevious+s6+v6*ts-amax*ts*ts/2+jmax*ts*ts*ts/6);
				_dist=(distPrevious+Math.abs(s6)+Math.abs(v6*ts-amax*ts*ts/2+jmax*ts*ts*ts/6));
				_vel=(v6-amax*ts+jmax*ts*ts/2);
			}
			i++;
			t=t+dt;

			if(i%100==0) {
				numSpeedPoints++;
				vel.add(_vel);
				dist2.add(_dist);   //total distance
				pos2.add(_pos);	  //encoder distance
				time.add(_time);
				
				
	// interpolate on the distance distance-angle pairs to find the matching angle, left wheel position
    // and right wheel position for the current distance. Then scale the velocity to get velL and velR
				index = Arrays.binarySearch(dist1.toArray(),index,dist1.size(),(_dist));
				if(index<0) index=-index;
				if(index>1)index=index-2;else index=0;
 
				if (index<=dist1.size()-2) {
					temp=(_dist-dist1.get(index))/(dist1.get(index+1) - dist1.get(index));
					angle = angle1.get(index)+( angle1.get(index+1) -angle1.get(index) )*temp;

					_dist2L = dist1L.get(index)+( dist1L.get(index+1) -dist1L.get(index) )*temp;
				
					_dist2R = dist1R.get(index)+( dist1R.get(index+1) -dist1R.get(index) )*temp;
				
					_posl = pos1L.get(index)+( pos1L.get(index+1) -pos1L.get(index) )*temp;
				
					_posr = pos1R.get(index)+( pos1R.get(index+1) -pos1R.get(index) )*temp;
				
				}
				else {
					angle=angle1.get(dist1.size()-1);
					_dist2L=dist1L.get(dist1.size()-1);
					_dist2R=dist1R.get(dist1.size()-1);
					_posl=pos1L.get(dist1.size()-1);
					_posr=pos1R.get(dist1.size()-1);
				}
				if(angle>999 || angle<-999)angle=angle2.get(angle2.size()-1);	
				if(_dist2L>999 || _dist2L<-999)angle=dist1L.get(dist1L.size()-1);	
				if(_dist2R>999 || _dist2R<-999)angle=dist1R.get(dist1R.size()-1);	
				if(_posr>999 || _posr<-999)angle=pos1R.get(pos1R.size()-1);	
				if(_posl>999 || _posl<-999)angle=pos1L.get(pos1L.size()-1);	


				angle2.add(angle);	
				dist2L.add(_dist2L);	
				dist2R.add(_dist2R);	
				pos2L.add(_posl);	
				pos2R.add(_posr);					

				if(dist2.size()>=2) {
					dpos2L=pos2L.get(pos2L.size()-1)-pos2L.get(pos2L.size()-2);
					dpos2R=pos2R.get(pos2R.size()-1)-pos2R.get(pos2R.size()-2);
					vl=dpos2L/(time.get(time.size()-1)-time.get(time.size()-2));
					vr=dpos2R/(time.get(time.size()-1)-time.get(time.size()-2));

				}
				else {
					dpos2L=dist2L.get(dist2L.size()-1);
					dpos2R=dist2R.get(dist2R.size()-1);
					vl=dpos2L/(time.get(time.size()-1));
					vr=dpos2R/(time.get(time.size()-1));
				}
				velL.add(vl);
				velR.add(vr);
				
				if (numSpeedPoints>=20000) {
					System.out.println("too many points, need to speed up!");
					break;
				}
			}
			
			
		}
	}	
		
	
	
	
	
	
	
	
	public void clearProfile(){
		px.clear();py.clear();angle1.clear();dist1.clear();pos1.clear();pdir.clear();
		pxL.clear();pyL.clear();dist1L.clear();pos1L.clear();
		pxR.clear();pyR.clear();dist1R.clear();pos1R.clear();
		time.clear();pos2.clear();vel.clear();dist2.clear();angle2.clear();
		pos2L.clear();velL.clear();dist2L.clear();
		pos2R.clear();velR.clear();dist2R.clear();
		pointsInProfile=0;
	}


	public void saveProfile() {
        int i = 0;
        int size = time.size();
        TrajectoryPoint point = new TrajectoryPoint();
        double angleConv=8192/(2*Math.PI);
        stream.Clear();
        while (i <size) {
           
                point.position =(int) (pos2.get(i) * Constants.kSensorUnitsPerInch);

                point.velocity = 0.0 * Constants.k_InchPerSecToVelUnits; 	
                point.arbFeedFwd =
                    0.5* (velL.get(i) + velR.get(i))*Constants.k_InchPerSecToVelUnits/Constants.maxVelUnitsPer100ms;    
                point.auxiliaryArbFeedFwd=
                0.5*(velL.get(i) - velR.get(i))*Constants.k_InchPerSecToVelUnits/Constants.maxVelUnitsPer100ms; 
                 	

                point.auxiliaryPos = (int)(angle2.get(i)*angleConv); /* scaled such that 3600 => 360 deg */
                point.profileSlotSelect0 = Constants.slot_pos; // slot for position
                point.profileSlotSelect1 = Constants.slot_angleMP; // slot for angle
                point.timeDur = 10;		
                point.useAuxPID = true;
                point.zeroPos = false;
                if (i == 0){
                    point.zeroPos = true; /* set this to true on the first point */
                }
                point.isLastPoint = false;
                if ( i == (size-1)){
                    point.isLastPoint = true; /* set this to true on the last point  */
                 }
                 stream.Write(point);
            i++;
            }
		
	}

	
	
	
	
}
