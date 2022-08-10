// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class DefaultIndexer extends CommandBase {

  IndexerSubsystem m_indexerSubsystem;

  /** Creates a new DefaultIndexer. */
  public DefaultIndexer(IndexerSubsystem indexerSubsystem) {
    addRequirements(indexerSubsystem);
    m_indexerSubsystem = indexerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getBallStatus(0) != BallStatus.kempty && getBallStatus(1) != BallStatus.kempty)
      m_indexerSubsystem.spinTransition(0.0);
    else
      m_indexerSubsystem.spinTransition(Constants.Indexer.transitionSpeed);
      /*
      if both hubs have no balls, spin
      else if upper hub has ball, and lower hub has no ball, spin
      else if upper hub has no ball, and lower hub has ball, spin

      else, both hubs have balls, no spin

      in shoot indexer, spin both the transition and belt to launch (easy launch ig)

      bleh - me
      */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.spinTransition(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
