/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.util;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Ramsete {

  public static RamseteCommand createRamseteCommand(Trajectory trajectory,
      DriveSubsystem driveSubsystem) {
    return new RamseteCommand(trajectory, driveSubsystem::getPose,
        new RamseteController(2, 0.7), driveSubsystem.getFeedforward(),
        driveSubsystem.getKinematics(), driveSubsystem::getSpeeds,
        driveSubsystem.getLeftPIDController(),
        driveSubsystem.getRightPIDController(), driveSubsystem::setOutput,
        driveSubsystem);
  }
}
