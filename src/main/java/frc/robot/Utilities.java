package frc.robot;

public class Utilities {
    
    //This program is developed in the hope of creating our own deadbands.
    //The Xbox Controller has small input errors that are fixed by this program.
    //We remove inputs between X which is listend as deadBandCutOff and -deadBandCutOff.
    protected double deadband(double value) {
      double deadband = 0.05;
      if (Math.abs(value) <= deadband) {
        return 0.0;
      } else {
        return value;
      }
    }

}
