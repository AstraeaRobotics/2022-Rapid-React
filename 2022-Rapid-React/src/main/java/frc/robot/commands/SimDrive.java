// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SimDrive extends CommandBase {

  SlewRateLimiter slewRateLimiter = new SlewRateLimiter(2.0);

  public SimDrive() {
    addRequirements(RobotContainer.m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double valetSpeed = 1;
    double leftAxis = RobotContainer.driverGamepad.getLeftX();
    double rightAxis = RobotContainer.driverGamepad.getRightX();
    double r2 = RobotContainer.driverGamepad.getR2Axis();
    double l2 = RobotContainer.driverGamepad.getL2Axis();

    double speed = (r2 - l2) * valetSpeed * (-1);
    double adjustedSpeed = slewRateLimiter.calculate(speed);

    RobotContainer.m_driveSubsystem.curveDrive(adjustedSpeed, leftAxis, false);

    if (Math.abs(rightAxis) > Constants.DriveConstants.DEADZONE) {
      RobotContainer.m_driveSubsystem.tankDriveAuto(rightAxis * Constants.DriveConstants.TURN_SPEED,
          -rightAxis * Constants.DriveConstants.TURN_SPEED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
