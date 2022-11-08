/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private double distanceMeters;
  private final boolean reverse;

  public DriveToDistance(DriveSubsystem driveSubsystem, double distanceMeters, boolean reverse) {
    addRequirements(driveSubsystem);
    this.m_DriveSubsystem = driveSubsystem;
    this.distanceMeters = distanceMeters;
    this.reverse = reverse;
  }

  @Override
  public void initialize() {
    m_DriveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    if(reverse) {
      m_DriveSubsystem.tankDrive(.6, .6);
    } else {
      m_DriveSubsystem.tankDrive(-.6, -.6);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_DriveSubsystem.getEncoderPosition()) > Math.abs(distanceMeters);
  }
}
