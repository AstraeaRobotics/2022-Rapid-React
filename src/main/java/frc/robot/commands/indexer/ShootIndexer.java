// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.status.Status;
import frc.robot.subsystems.IndexerSubsystem;

public class ShootIndexer extends CommandBase {

  IndexerSubsystem m_indexerSubsystem;

  /** Creates a new ShootIndexer. */
  public ShootIndexer(IndexerSubsystem indexerSubsystem) {
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
    m_indexerSubsystem.spinBelt(Constants.Indexer.beltSpeed);
    m_indexerSubsystem.spinTransition(Constants.Indexer.transitionSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.spinBelt(0.0);
    m_indexerSubsystem.spinTransition(0.0);
  }
}
