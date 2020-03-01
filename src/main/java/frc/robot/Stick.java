package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

public class Stick extends Joystick{
    public double leftinput=0,rightinput=0, forward,turn;
    private boolean[] _button,button;
    int numButtons;
    double deadband = 0.05;
    double xPrev = 0;
    double forwardSF=0.7,turnSF=0.5;

    Stick(int portnum){
        super(portnum);       
        numButtons = this.getButtonCount();  // ??? This didn't work on switch joystick
        numButtons=11;
        _button = new boolean[numButtons];
        button = new boolean[numButtons];

    }
    public double getRampedInput() {
        double x,y;
        double deltaXMax=Constants.oneWayRampTime;
        x = -this.getRawAxis(1);
        double deltaX = x - xPrev;
        if (this.getRawButton(1)){ y = x;}
        else{
    
        if (deltaX > deltaXMax && x > 0){
            deltaX = deltaXMax;
        }
        else if (deltaX < -deltaXMax && x < 0) {
            deltaX = -deltaXMax;
        }
        else if(deltaX == 0){
            y = x;
        }
        y = xPrev + deltaX;
        xPrev = y;
     }
     return(y);

}

    public void readDriverStick(){
        readDriverJoy();
        readButtons();
    }

    public void readOperatorStick(){
        readOperatorJoy();
        readButtons();
    }



    public void readDriverJoy() {
    // get the forward and turn axis, and convert to left and right motor inputs        
    // square the inputs for enhanced low speed control and apply deadband  
        double sf=Constants.forwardSF;
        if(Robot.turbo)sf=1;
//        forward=-getRampedInput()*sf;
        forward = -this.getRawAxis(1)*sf;  
        if (forward<deadband && forward>-deadband)forward=0;
        forward = Math.pow(forward,2)*Math.signum(forward);
        turn =this.getRawAxis(2)*Constants.turnSF;
        if (turn<deadband && turn>-deadband)turn=0;
        turn = Math.pow(turn,2)*Math.signum(turn);

        leftinput=forward+turn;
        rightinput=forward-turn;

        // limit input to +/- 1    
        leftinput=limit(leftinput);
        rightinput=limit(rightinput);
    }

    public void readOperatorJoy() {
            forward = -this.getRawAxis(1);           
            if (forward<deadband && forward>-deadband)forward=0;   
            turn =0;
        }
    


    public void readButtons(){
    // read the buttons
        int i=1;
        while(i<numButtons){
            _button[i]=button[i];
            button[i]=this.getRawButton(i);
            i++;
        }
    }

    public boolean getButton(int num){
        return this.button[num];
    }

    public boolean getPrevButton(int num){
        return this._button[num];
    }

    public double getLeft(){
        return this.leftinput;
    }

    public double getRight(){
        return this.rightinput;
    }

    public double limit(double value) {
        return Math.max(-1.0, Math.min(value, 1.0));
    }

    public boolean isPushed(){
        readDriverStick();
        boolean leftpush = (Math.abs(this.leftinput)>0.05) ; 
        boolean rightpush = (Math.abs(this.rightinput)>0.05) ; 
        return ((leftpush||rightpush));
    }


}