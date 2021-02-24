// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/** This is a demo program showing how to use Mecanum control with the RobotDrive class. */
public class Robot extends TimedRobot {

  private MecanumDrive m_robotDrive;
  private XboxController controller;

  @Override
  public void robotInit() {
    CANSparkMax frontLeft = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(2, MotorType.kBrushless);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    controller = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick raw axis 1 for fowards and backwards movement
    // Use the joystick raw axis 0 for left and right movement
    //Use the joystick raw axis 4 for strafe movement
    m_robotDrive.driveCartesian(controller.getY(Hand.kLeft), controller.getX(Hand.kLeft), controller.getX(Hand.kRight), 0);
}
}
