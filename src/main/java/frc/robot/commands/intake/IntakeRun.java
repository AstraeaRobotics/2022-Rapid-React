/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.indexer.RejectBall;
import frc.robot.status.Status;
import frc.robot.status.Status.IntakeStatus;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeRun. */
  IntakeSubsystem m_IntakeSubsystem;
  Alliance team;
  IndexerSubsystem m_indexerSubsystem;
  RejectBall m_rejectball;

  public IntakeRun(IntakeSubsystem system, IndexerSubsystem system2) {
    m_IntakeSubsystem = system;
    m_indexerSubsystem = system2;
    addRequirements(m_IntakeSubsystem);
    team = DriverStation.getAlliance();
    m_rejectball = new RejectBall(m_IntakeSubsystem, m_indexerSubsystem, 1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // motors are moving forwards
    if (m_IntakeSubsystem.getBallColor() != Alliance.Invalid
        && m_IntakeSubsystem.getBallColor() != DriverStation.getAlliance()) {
      SmartDashboard.putBoolean("Rejection", true);
      SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
      CommandScheduler.getInstance().schedule(m_rejectball);
    } else if (m_IntakeSubsystem.isExtended()) {
      Status.logIntakeStatus(IntakeStatus.kExtended);
      SmartDashboard.putBoolean("Rejection", false);
      m_IntakeSubsystem.setMotor(0.5);
    } else {
      Status.logIntakeStatus(IntakeStatus.kRetracted);
      m_IntakeSubsystem.setMotor(0);
    }

    // if (m_IntakeSubsystem.isExtended()) {
    // Status.logIntakeStatus(IntakeStatus.kExtended);
    // m_IntakeSubsystem.setMotor(0.5);
    // } else {
    // Status.logIntakeStatus(IntakeStatus.kRetracted);
    // m_IntakeSubsystem.setMotor(0);
    // }
    // }
  }
}
