/********************************************************************************
*                                                                               *
*   Copyright (c) Astraea Robotics, FIRST, and other WPILib contributors        *
*                                                                               *
*   Open Source Software; you can modify and/or share it under the terms of     *
*   the license file in the root directory of this project.                     *
*                                                                               *
********************************************************************************/
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.indexer.ShootIndexer;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TwoBall extends SequentialCommandGroup {
  /** Creates a new OneBall. */
  public TwoBall(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    addCommands(
      new ToggleIntake(intakeSubsystem),
      new DriveToDistance(driveSubsystem, 1),
      new ParallelDeadlineGroup(new WaitCommand(2.0), new IntakeRun(intakeSubsystem)),
      new TurnToAngle(180, driveSubsystem),
      new DriveToDistance(driveSubsystem, 1),
      new ParallelDeadlineGroup(new WaitCommand(1.0), new ShootIndexer(indexerSubsystem))
    );
  }
}
