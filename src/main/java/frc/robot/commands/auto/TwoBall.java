// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.turret.ToggleTurretLock;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public TwoBall(DriveSubsystem drive, TurretSubsystem turret, IndexerSubsystem indexer, IntakeSubsystem intake) {
    addCommands(
      new ToggleTurretLock(turret),
      new ToggleIntake(intake), 
      new ParallelDeadlineGroup(
        new DriveToDistance(drive, 1, false),
        new IntakeRun(intake)
      ),
      new ToggleIntake(intake),
      new WaitCommand(2),
      new TurnToAngle(180, drive),
      new DriveToDistance(drive, 1, false)
    );
  }
}
