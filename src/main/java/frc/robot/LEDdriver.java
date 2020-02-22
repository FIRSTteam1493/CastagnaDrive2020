package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;;

public class LEDdriver{
    DigitalOutput do1 = new DigitalOutput(23);
    DigitalOutput do2 = new DigitalOutput(22);
    DigitalOutput do3 = new DigitalOutput(21);

    public LEDdriver(){    
    }
    
    public void sendData(int i){
      if(i==0) {do1.set(false);do2.set(false);do3.set(false);}
      else if(i==1){do1.set(false);do2.set(false);do3.set(true);}
      else if(i==2){do1.set(false);do2.set(true);do3.set(false);}
      else if(i==3){do1.set(false);do2.set(true);do3.set(true);}
      else if(i==4){do1.set(true);do2.set(false);do3.set(false);}
      else if(i==5){do1.set(true);do2.set(false);do3.set(true);}
      else if(i==6){do1.set(true);do2.set(true);do3.set(false);}
      else if(i==7){do1.set(true);do2.set(true);do3.set(true);}
    }
}
    


