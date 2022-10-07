/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private double distanceMeters;
  private final int reverseFactor;

  private final double desiredDistance;

  public DriveToDistance(DriveSubsystem driveSubsystem, double distanceMeters) {
    addRequirements(driveSubsystem);
    this.m_DriveSubsystem = driveSubsystem;
    this.reverseFactor = distanceMeters > 0 ? 1 : -1;
    this.desiredDistance = distanceMeters;
  }

  @Override
  public void execute() {
    //FIXME: .4 is magic number
    m_DriveSubsystem.tankDrive(.4 * reverseFactor, .4 * reverseFactor);
  }

  @Override
  public void initialize() {
    distanceMeters = desiredDistance + m_DriveSubsystem.getEncoderPosition();
  }

  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.tankDrive(0, 0);
  }

  /*
   * If distanceMeters is positive, then the robot will drive forward. 
   *    To end the command, the robot must have driven forward, and the distance in meters must be greater by distancemeters or more
   * If distanceMeters is negative, then the robot will drive backward. Same thing for less
   */

  @Override
  public boolean isFinished() {
    if (reverseFactor == 1) {
      return distanceMeters <= m_DriveSubsystem.getEncoderPosition();
    }
    //Moving Backwards
    return distanceMeters >= m_DriveSubsystem.getEncoderPosition();
  }
}
