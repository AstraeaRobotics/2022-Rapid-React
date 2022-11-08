/********************************************************************************
 * * Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors * * Open Source Software;
 * you can modify and/or share it under the terms of * the license file in the root directory of
 * this project. * *
 ********************************************************************************/
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeRun. */
  IntakeSubsystem m_IntakeSubsystem;

  public IntakeRun(IntakeSubsystem system) {
    m_IntakeSubsystem = system;
    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void execute() {
    if (m_IntakeSubsystem.isExtended()) {
      Status.logIntakeStatus(IntakeStatus.kExtended);
      m_IntakeSubsystem.setMotor(0.5);
    } else {
      Status.logIntakeStatus(IntakeStatus.kRetracted);

      m_IntakeSubsystem.setMotor(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setMotor(0);
  }
}
