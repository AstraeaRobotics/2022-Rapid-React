// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class ReverseIndexer extends CommandBase {
  IndexerSubsystem m_indexerSubsystem;

  public ReverseIndexer(IndexerSubsystem indexerSubsystem) {
    addRequirements(indexerSubsystem);
    m_indexerSubsystem = indexerSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerSubsystem.spinBelt(-Constants.Indexer.kBeltSpeed);
    m_indexerSubsystem.spinTransition(-Constants.Indexer.kTransitionSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.spinBelt(0.0);
    m_indexerSubsystem.spinTransition(0.0);
  }
}
