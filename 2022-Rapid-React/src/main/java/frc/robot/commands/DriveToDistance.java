// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final double distance;

  /** Creates a new DriveCommand. */
  public DriveToDistance(DriveSubsystem driveSubsystem, double distance) { 
    m_driveSubsystem = driveSubsystem;
    this.distance = distance;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_driveSubsystem.tankDrive(1, 1);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.tankDrive(0, 0);

  }

  @Override
  public boolean isFinished() {
    return m_driveSubsystem.getEncoderPosition() > distance;
  }
}
