package frc.robot;

public class Utilities {
    
    public double getDeadBand(double Joystick, double deadbandCutOff){
        if(Joystick < deadbandCutOff && Joystick > (deadbandCutOff*-1)){
          return Joystick=0;
        } else {
          return Joystick = (Joystick-(Math.abs(Joystick)/Joystick*deadbandCutOff))/(1-deadbandCutOff);
        }
      }

}
