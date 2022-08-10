// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends CommandBase {

  IndexerSubsystem m_indexerSubsystem;

  public RunIndexer(IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerSubsystem);
    m_indexerSubsystem = indexerSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerSubsystem.spinTransition(0.5);
    m_indexerSubsystem.spinBelt(0.5);

    SmartDashboard.putNumber("Indexer Ball Proximity", m_indexerSubsystem.getProximity());
    SmartDashboard.putString("Indexer Ball Color", m_indexerSubsystem.getDetectedColor().toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.spinTransition(0.0);
    m_indexerSubsystem.spinBelt(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (upperBall == null && lowerBall == null)
      return true;
    return false;
  }
}
