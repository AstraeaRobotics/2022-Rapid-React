// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.indexer.RejectBall;

public class RunIndexer extends CommandBase {

  IndexerSubsystem m_indexerSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  public RunIndexer(IndexerSubsystem indexerSubsystem, IntakeSubsystem system) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexerSubsystem = indexerSubsystem;
    m_IntakeSubsystem = system;
    addRequirements(m_indexerSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // motors are moving forwards
    if (m_indexerSubsystem.getBallColor() != Alliance.Invalid && m_indexerSubsystem.getBallColor() != DriverStation.getAlliance()){
      new RejectBall(m_IntakeSubsystem, m_indexerSubsystem, 3);
    }
    else{
      m_indexerSubsystem.spinMotors(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.spinMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
