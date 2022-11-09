// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Limelight;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */

  DriveSubsystem m_driveSubsystem;
  double y_offset;
  boolean close;

  public AutoAim(DriveSubsystem m_driveSubsystem) {

    this.m_driveSubsystem = m_driveSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    close = false;
    y_offset = Limelight.getTy();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    y_offset = Limelight.getTy();

    if (y_offset > 1) {
      m_driveSubsystem.tankDrive(0.7, 0.7);
    } else if (y_offset < -1) {
      m_driveSubsystem.tankDrive(-0.7, -0.7);
    } else {
      close = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return close;
  }
}
