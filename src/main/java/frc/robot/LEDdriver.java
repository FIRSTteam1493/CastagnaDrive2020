package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;;

public class LEDdriver{
    DigitalOutput do1 = new DigitalOutput(1);
    DigitalOutput do2 = new DigitalOutput(2);
    DigitalOutput do3 = new DigitalOutput(3);

    public LEDdriver(){    
    }
    
    public void sendData(int i){
        boolean b1=false,b2=false,b3=false;
        int i1=i/4;
        int i2 = (i/2)%2;
        int i3 = i%2;
        b1=(i1==1);
        b1=(i2==1);
        b1=(i3==1);
        do1.set(b1);
        do2.set(b2);
        do3.set(b3);
    }
}
    


/*
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class LEDdriver{
I2C Wire = new I2C(Port.kOnboard, 4);
public LEDdriver(){

}

public void sendData(int i){
    byte[] b = new byte[2];
    b[0]=1;
    b[1]=2;
	Wire.writeBulk(b, 2);
}
}

*/