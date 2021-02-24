// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/** This is a demo program showing how to use Mecanum control with the RobotDrive class. */
public class Robot extends TimedRobot {

  private MecanumDrive m_robotDrive;
  private XboxController controller;
  private Utilities utils;

  private double FLVoltage;
  private double BLVoltage;
  private double FRVoltage;
  private double BRVoltage;

  @Override
  public void robotInit() {
    //Giving Spark Maxs there motor positions and the intiger that is assosciated with them.
    //To change Spark Max Intigers use Rev Hardware Client found on Desktop
    //MotorType is the tpye of motor we are using with the motor controller.
    //If you wish to attatch a motor that is not burshless make sure to connect to Spark Max with USB C cord and adjust that as neccessary in Rev Hardware Client
    CANSparkMax frontLeft = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(2, MotorType.kBrushless);

    //These are to return the voltage values we are getting for each motor to the SmartDashboard
    FLVoltage = frontLeft.getBusVoltage();
    BLVoltage = frontLeft.getBusVoltage();
    FRVoltage = frontLeft.getBusVoltage();
    BRVoltage = frontLeft.getBusVoltage();

    //Invert motor direction
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    //giving m_robotDrive the motors it asks for
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    //The Xbox Controller and the input intiger that corresponds with its usb port
    controller = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick raw axis 1 for fowards and backwards movement
    // Use the joystick raw axis 0 for left and right movement
    //Use the joystick raw axis 4 for strafe movement
    //This code was working originally but needed to be altared for a deadband fix for the controllers
    //m_robotDrive.driveCartesian(controller.getY(Hand.kLeft), controller.getX(Hand.kLeft), controller.getX(Hand.kRight), 0);

    //This is untested Code for trying to implement a deadband for the Xbox controller on our 2021 mecanum drive robot
    //THe deadband Utility can be found under Utilities.java

    //The getDeadBand funtion will take joystick input and prevent that value from moving the robot if user input is lower than required value which is currently .2
    m_robotDrive.driveCartesian(utils.getDeadBand(controller.getY(Hand.kLeft), .2), utils.getDeadBand(controller.getX(Hand.kLeft), .2), utils.getDeadBand(controller.getX(Hand.kRight), .2), 0);

    //This is where we will put things we wish to observe on the dashboard.
    //SmartDashboard is the utility we wish to use

    //These Specifically send the voltage that each motor controller is recieiving to the SmartDashboard
    SmartDashboard.putNumber("Front Left Voltage", FLVoltage);
    SmartDashboard.putNumber("Back Left Voltage", BLVoltage);
    SmartDashboard.putNumber("Front Right Voltage", FRVoltage);
    SmartDashboard.putNumber("Back Right Voltage", BRVoltage);
}
}
