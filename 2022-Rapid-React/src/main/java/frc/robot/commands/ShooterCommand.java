// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  public ShooterCommand(VisionSubsystem mVisionsubsystem, ShooterSubsystem sSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mVisionsubsystem);
    addRequirements(sSubsystem);
    this.visionSubsystem = mVisionsubsystem;
    this.shooterSubsystem = sSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setMotors(0.5,0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotors();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
