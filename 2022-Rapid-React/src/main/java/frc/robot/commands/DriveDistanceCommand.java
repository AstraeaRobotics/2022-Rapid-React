// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {

  DriveSubsystem subsystem;
  double distance;
  /** Creates a new DriveCommand. */
  public DriveDistanceCommand( DriveSubsystem drive, double distance) { 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    subsystem = drive;
    this.distance = distance*(Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_DIAMETER * math.pi);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    subsystem.tankDrive(0.25, 0.25);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.tankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
