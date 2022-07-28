// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeRun. */
  IntakeSubsystem m_IntakeSubsystem;

  public IntakeRun(IntakeSubsystem system) {
    m_IntakeSubsystem = system;
    addRequirements(m_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_IntakeSubsystem.isExtended()) {
      m_IntakeSubsystem.setMotor(0.5);
    } else {
      m_IntakeSubsystem.setMotor(0);
    }
  }
}
