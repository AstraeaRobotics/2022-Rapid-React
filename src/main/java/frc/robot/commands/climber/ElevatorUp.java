// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants;

public class ElevatorUp extends CommandBase {
  /** Creates a new Elevator. */
  private ClimberSubsystem m_climberSubsystem;

  public ElevatorUp(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
    if (!m_climberSubsystem.isFullyRetracted()) {
      m_climberSubsystem.setSpeed(speed);
    } else {
      m_climberSubsystem.setSpeed(0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
