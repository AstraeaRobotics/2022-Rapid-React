// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {


  private final DriveSubsystem m_DriveSubsystem;
  private final double distanceMeters;

  public DriveToDistance(DriveSubsystem driveSubsystem, double distanceMeters) {
    addRequirements(driveSubsystem);
    this.m_DriveSubsystem = driveSubsystem;
    this.distanceMeters = distanceMeters + m_DriveSubsystem.getEncoderPosition();
  }

  @Override
  public void execute() {
    m_DriveSubsystem.tankDrive(.4, .4);
    System.out.println("RUNNING IT");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("STOPPED");
    m_DriveSubsystem.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_DriveSubsystem.getEncoderPosition() > distanceMeters;
  }
}
