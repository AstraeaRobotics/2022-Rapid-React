// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.ReverseIndexer;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReverseAll extends ParallelCommandGroup {
  /** Creates a new ReverseAll. */
  public ReverseAll(IntakeSubsystem intakeSubsystem,
      IndexerSubsystem indexerSubsystem) {
    addCommands(new ReverseIntake(intakeSubsystem),
        new ReverseIndexer(indexerSubsystem));
  }
}
