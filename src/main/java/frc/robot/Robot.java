// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

  private XboxController controller;
  private Utilities utils;

  private double FLVoltage;
  private double BLVoltage;
  private double FRVoltage;
  private double BRVoltage;

  MecanumDriveKinematics m_kinematics;

  CANSparkMax frontLeft;
  CANSparkMax rearLeft;
  CANSparkMax frontRight;
  CANSparkMax rearRight;

  @Override
  public void robotInit() {
    //Giving Spark Maxs there motor positions and the intiger that is assosciated with them.
    //To change Spark Max Intigers use Rev Hardware Client found on Desktop
    //MotorType is the tpye of motor we are using with the motor controller.
    frontLeft = new CANSparkMax(4, MotorType.kBrushless);
    rearLeft = new CANSparkMax(5, MotorType.kBrushless);
    frontRight = new CANSparkMax(1, MotorType.kBrushless);
    rearRight = new CANSparkMax(2, MotorType.kBrushless);

    utils = new Utilities();

    //These are to return the voltage values we are getting for each motor to the SmartDashboard
    FLVoltage = frontLeft.getBusVoltage();
    BLVoltage = frontLeft.getBusVoltage();
    FRVoltage = frontLeft.getBusVoltage();
    BRVoltage = frontLeft.getBusVoltage();

    //Invert motor direction
    frontRight.setInverted(false);
    frontLeft.setInverted(true);
    rearRight.setInverted(false);
    rearLeft.setInverted(true);

    //The Xbox Controller and the input intiger that corresponds with its usb port.
    controller = new XboxController(0);

    double widthValue = 0.26;
    double lengthValue = 0.29;
    Translation2d m_frontLeftLocation = new Translation2d(lengthValue, widthValue);
    Translation2d m_frontRightLocation = new Translation2d(lengthValue, -1*widthValue);
    Translation2d m_backLeftLocation = new Translation2d(-1*lengthValue, widthValue);
    Translation2d m_backRightLocation = new Translation2d(-1*lengthValue, -1*widthValue);

    // Creating my kinematics object using the wheel locations.
    m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

  }

  @Override
  public void teleopPeriodic() {
    /*Use the joystick raw axis 1 for fowards and backwards movement
    Use the joystick raw axis 0 for left and right movement
    Use the joystick raw axis 4 for strafe movement*/

    //This is untested Code for trying to implement a deadband for the Xbox controller
    //The deadband Utility can be found under Utilities.java

    //The getDeadBand funtion will take joystick input and prevent that value from moving the robot if user input is within given range which is currently .2 and -.2
    double speedFactor = 0.50;
    double leftY = utils.deadband(controller.getY(Hand.kLeft)*speedFactor);
    double leftX = utils.deadband(controller.getX(Hand.kLeft)*speedFactor);
    double rightX = utils.deadband(controller.getX(Hand.kRight)*speedFactor);
    
    ChassisSpeeds speeds = new ChassisSpeeds(leftY, leftX, rightX);    
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    double spFrontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double spFrontRight = wheelSpeeds.frontRightMetersPerSecond;
    double spBackLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double spBackRight = wheelSpeeds.rearRightMetersPerSecond;

    frontLeft.set(spFrontLeft);
    frontRight.set(spFrontRight);
    rearLeft.set(spBackLeft);
    rearRight.set(spBackRight);
    
    //This is where we will put things we wish to observe on the dashboard.
    //SmartDashboard is the utility we wish to use

    //These send the voltage that each motor controller is recieiving to the SmartDashboard
    SmartDashboard.putNumber("Front Left Voltage", FLVoltage);
    SmartDashboard.putNumber("Back Left Voltage", BLVoltage);
    SmartDashboard.putNumber("Front Right Voltage", FRVoltage);
    SmartDashboard.putNumber("Back Right Voltage", BRVoltage);
}
}
