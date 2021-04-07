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
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class Robot extends TimedRobot {

  private XboxController controller;
  private Utilities utils;

  private double FLVoltage;
  private double BLVoltage;
  private double FRVoltage;
  private double BRVoltage;

  private double LSVoltage;
  private double RSVoltage;

  private CANDigitalInput LSForwardLimit;
  private CANDigitalInput LSReverseLimit;
  private CANDigitalInput RSForwardLimit;
  private CANDigitalInput RSReverseLimit;

  public String kEnable;
  public String kDisable;

  MecanumDriveKinematics m_kinematics;

  CANSparkMax frontLeft;
  CANSparkMax rearLeft;
  CANSparkMax frontRight;
  CANSparkMax rearRight;

  CANSparkMax leftShooter;
  CANSparkMax rightShooter;


  @Override
  public void robotInit() {
    //The Intiger provided is found by connecting to the Spark Max on the Rev Hardware Client with the USB C Cable.
    //MotorType is based on the motor we are using. NEO Brushless is a MotorType.kBrushless.
    frontLeft = new CANSparkMax(4, MotorType.kBrushless);
    rearLeft = new CANSparkMax(5, MotorType.kBrushless);
    frontRight = new CANSparkMax(1, MotorType.kBrushless);
    rearRight = new CANSparkMax(2, MotorType.kBrushless);

    leftShooter = new CANSparkMax(6, MotorType.kBrushless);
    rightShooter = new CANSparkMax(7, MotorType.kBrushless);

    utils = new Utilities();

    //These are to return the Voltage Values from motors
    FLVoltage = frontLeft.getBusVoltage();
    BLVoltage = frontLeft.getBusVoltage();
    FRVoltage = frontLeft.getBusVoltage();
    BRVoltage = frontLeft.getBusVoltage();

    LSVoltage = leftShooter.getBusVoltage();
    RSVoltage = leftShooter.getBusVoltage();

    //Invert motor directions
    frontRight.setInverted(false);
    frontLeft.setInverted(true);
    rearRight.setInverted(false);
    rearLeft.setInverted(true);

    leftShooter.setInverted(true);
    rightShooter.setInverted(false);

    //The Xbox Controller and the intiger provided by the Driver Station
    controller = new XboxController(0);

    //Actual Mecanum Math
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

    //The limit switch examples provided by Rev Robotics
    LSForwardLimit = leftShooter.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
    LSReverseLimit = leftShooter.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);

    LSForwardLimit.enableLimitSwitch(false);
    LSReverseLimit.enableLimitSwitch(false);
    SmartDashboard.putBoolean("Left Shooter Forward Limit Enabled", LSForwardLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Left Shooter Reverse Limit Enabled", LSReverseLimit.isLimitSwitchEnabled());

    RSForwardLimit = rightShooter.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
    RSReverseLimit = rightShooter.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);

    RSForwardLimit.enableLimitSwitch(false);
    RSReverseLimit.enableLimitSwitch(false);
    SmartDashboard.putBoolean("Right Shooter Forward Limit Enabled", RSForwardLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Right Shooter Reverse Limit Enabled", RSReverseLimit.isLimitSwitchEnabled());
  }

  @Override
  public void teleopPeriodic() {
    /*Use the joystick raw axis 1 for fowards and backwards movement
    Use the joystick raw axis 0 for left and right movement
    Use the joystick raw axis 4 for strafe movement*/

    //The deadband Utility can be found under Utilities.java

    //The getDeadBand funtion will take joystick input and prevent that value from moving the robot if user input is within given range which is currently .2 and -.2
    //speedFactor is to adjust the speed of the Robot.
    double speedFactor = 0.50;
    double shooterspeedFactor = 0.50;
    double leftY = utils.deadband(controller.getY(Hand.kLeft)*speedFactor);
    double leftX = utils.deadband(controller.getX(Hand.kLeft)*speedFactor);
    double rightX = utils.deadband(controller.getX(Hand.kRight)*speedFactor);
    double rightY = utils.deadband(controller.getY(Hand.kRight)*shooterspeedFactor);
    
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

    leftShooter.set(rightY);
    rightShooter.set(rightY);

    LSForwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Left Shooter Forward Limit Enabled", false));
    LSReverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Left Shooter Reverse Limit Enabled", false));
    RSForwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Right Shooter Forward Limit Enabled", false));
    RSReverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Right Shooter Reverse Limit Enabled", false));

    //This is where we will put things we wish to observe on the dashboard.
    //SmartDashboard is the utility we wish to use

    //These send the voltage that each motor controller is recieiving to the SmartDashboard
    SmartDashboard.putNumber("Front Left Voltage", FLVoltage);
    SmartDashboard.putNumber("Back Left Voltage", BLVoltage);
    SmartDashboard.putNumber("Front Right Voltage", FRVoltage);
    SmartDashboard.putNumber("Back Right Voltage", BRVoltage);

    SmartDashboard.putNumber("Left Shooter Voltage", LSVoltage);
    SmartDashboard.putNumber("Right Shooter Voltage", RSVoltage);

    SmartDashboard.putBoolean("Left Shooter Forward Limit Switch", LSForwardLimit.get());
    SmartDashboard.putBoolean("Left Shooter Reverse Limit Switch", LSReverseLimit.get());
    SmartDashboard.putBoolean("Right Shooter Forward Limit Switch", LSForwardLimit.get());
    SmartDashboard.putBoolean("Right Shooter Reverse Limit Switch", LSReverseLimit.get());
}
}
