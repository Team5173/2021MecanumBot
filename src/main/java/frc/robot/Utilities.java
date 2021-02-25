package frc.robot;

public class Utilities {
    
    //This program is developed in the hope of creating our own deadbands.
    //The Xbox Controller has small input errors that are fixed by this program.
    //We remove inputs between X which is listend as deadBandCutOff and -deadBandCutOff.
    public double getDeadBand(double Joystick, double deadbandCutOff){
        if((Joystick > deadbandCutOff) || (Joystick < -deadbandCutOff)){
          return Joystick;
        } else {
          return 0;
        }
      }

}
