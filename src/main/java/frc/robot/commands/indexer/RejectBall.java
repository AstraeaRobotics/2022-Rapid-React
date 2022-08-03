// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.status.Status.IndexerStatus;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RejectBall extends WaitCommand {
  IntakeSubsystem m_IntakeSubsystem;
  IndexerSubsystem m_IndexerSubsystem;
  Alliance team;

  /** Creates a new IntakeReject. */
  public RejectBall(IntakeSubsystem system, IndexerSubsystem system2, double seconds) {
    super(seconds);
    m_IntakeSubsystem = system;
    m_IndexerSubsystem = system2;
    team = DriverStation.getAlliance();
    addRequirements(m_IntakeSubsystem, m_IndexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setMotor(-0.5);
    m_IndexerSubsystem.spinMotors(-0.5);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_IntakeSubsystem.setMotor(0);
    m_IndexerSubsystem.spinMotors(0);
  }
}
