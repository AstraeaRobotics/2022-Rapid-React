// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexer.ShootIndexer;
import frc.robot.commands.turret.ToggleTurretLock;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBall extends SequentialCommandGroup {
  /** Creates a new OneBall. */
  public OneBall(TurretSubsystem turret, DriveSubsystem drive, IndexerSubsystem indexer) {
    addCommands(
      new ToggleTurretLock(turret),
      new DriveToDistance(drive, 1.33, true),
      new ParallelDeadlineGroup(new WaitCommand(3), new ShootIndexer(indexer)),
      new ToggleTurretLock(turret),
      new DriveToDistance(drive, 2, true)
    );
  }
}
