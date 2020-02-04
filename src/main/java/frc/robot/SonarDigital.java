/*
From Maxbotix HLRX MaxSonar EZ datasheet:  By default, the serial output is RS232 format (0 to Vcc) with a 1-mm resolution. If TTL
output is desired, solder the TTL jumper pads on the back side of the PCB as shown in the photo to the right.
For volume orders, the TTL option is available as no-cost factory installed jumper. The output is an ASCII
capital “R”, followed by four ASCII character digits representing the range in millimeters, followed by a
carriage return (ASCII 13). The maximum distance reported is 5000. The serial output is the most accurate of the range
outputs. Serial data sent is 9600 baud, with 8 data bits, no parity, and one stop bit.

    binary      dec     hex
R = 01010010    82      52 
0 = 00110000    48      30
1 = 00110001    49      31  
2 = 00110010    50      32
3 = 00110011    51      33
4 = 00110100    52      34
5 = 00110101    53      35
6 = 00110110    54      36
7 = 00110111    55      37
8 = 00111000    56      38
9 = 00111001    57      39
CR= 00001101    13      0D

The java byte data type holds 8 bits - 1 character
*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

public class SonarDigital {
    private SerialPort serial = new SerialPort(9600, Port.kMXP, 8, Parity.kNone, StopBits.kOne);
    private byte[] bytes = new byte[6];
    private String val, r, str;
    private double distance = 0;
    Notifier notifier;

    SonarDigital() {
        (new Thread() {
            public void run() {
                while (true) {
                    try {
                        Thread.sleep(10);  // poll the serial port every 10 ms
                    } catch (InterruptedException e) {
                        System.out.println("Digital sonar error");
                    }
                measureDistance();
            }
            }
           }).start();

    }

    private void measureDistance(){
        if(serial.getBytesReceived() >=6) {
            bytes=serial.read(6);
            str = String.valueOf(bytes);
            r=str.substring(0, 1);
           val=str.substring(1,5);
           distance = 40.2755*(Integer.parseInt(val));
        }
    }

    public double getDistance(){
        return distance;
    }

    public String getFirstChar(){
        return r;
    }
}