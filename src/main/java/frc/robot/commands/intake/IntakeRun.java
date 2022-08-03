// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeRun. */
  IntakeSubsystem m_IntakeSubsystem;
  Alliance team;

  public IntakeRun(IntakeSubsystem system) {
    m_IntakeSubsystem = system;
    addRequirements(m_IntakeSubsystem);
    team = DriverStation.getAlliance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_IntakeSubsystem.isExtended()) {
      Status.logIntakeStatus(IntakeStatus.kExtended);
      if (m_IntakeSubsystem.getRejectState() == false) {
        m_IntakeSubsystem.setMotor(0.5);
      }
    } else {
      Status.logIntakeStatus(IntakeStatus.kRetracted);
      m_IntakeSubsystem.setMotor(0);
    }
  }
}