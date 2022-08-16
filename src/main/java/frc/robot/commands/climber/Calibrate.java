// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

// for setting soft limits
public class Calibrate extends CommandBase {
  /** Creates a new Calibrate. */
  private ClimberSubsystem m_climberSubsystem;

  public Calibrate(ClimberSubsystem climbSubsystem) {
    m_climberSubsystem = climbSubsystem;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.setSpeed(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
    m_climberSubsystem.reset();
    m_climberSubsystem.setSoftLimits();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climberSubsystem.isFullyRetracted();
  }
}
