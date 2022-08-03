// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBall. */

  public TwoBall(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(drive, Traj.createNewTrajectoryFromJSON("TwoBall-1")),
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("TwoBall-1"), drive, true),
      new ParallelCommandGroup(
        new ToggleIntake(intake),
        new DriveToDistance(drive, 0.5)
      ),
      new ParallelCommandGroup(
        new ToggleIntake(intake),
        new DriveToDistance(drive, 0.5)
      ),
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("TwoBall-2"), drive, true),
      new ParallelDeadlineGroup(new WaitCommand(1), new ManualShoot(shooter, 50, 50))
    );
  }
}
