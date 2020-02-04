package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

public class Stick extends Joystick{
    private double leftinput=0,rightinput=0;
    private boolean[] _button,button;
    int numButtons;
    double deadband = 0.05;

    Stick(int portnum){
        super(portnum);       
        numButtons = this.getButtonCount();  // ??? This didn't work on switch joystick
        numButtons=10;
        _button = new boolean[numButtons];
        button = new boolean[numButtons];
    }

    public void readStick(){
        readJoy();
        readButtons();
    }

    public void readJoy() {
    // get the forward and turn axis, and convert to left and right motor inputs        
    // square the inputs for enhanced low speed control and apply deadband  
        double forward = -this.getRawAxis(1);  
        if (forward<deadband && forward>-deadband)forward=0;
        forward = Math.pow(forward,2)*Math.signum(forward);
        double turn =this.getRawAxis(2);
        if (turn<deadband && turn>-deadband)turn=0;
        turn = Math.pow(turn,2)*Math.signum(turn);

        leftinput=forward-turn;
        rightinput=forward+turn;

        // limit input to +/- 1    
        leftinput=limit(leftinput);
        rightinput=limit(rightinput);
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
        readStick();
        boolean leftpush = (Math.abs(this.leftinput)>0.1) ; 
        boolean rightpush = (Math.abs(this.rightinput)>0.1) ; 
        return ((leftpush||rightpush));
    }


}