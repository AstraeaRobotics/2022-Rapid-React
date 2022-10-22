// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.drive.Teleport;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.Constants;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.intake.IntakeRun;
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
public class ThreeBall extends SequentialCommandGroup {
  /** Creates a new ThreeBall. */

  public ThreeBall(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ResetOdometry(drive, Traj.createNewTrajectoryFromJSON("ThreeBall-1")),
      // Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-1"), drive, true),
      // new ParallelCommandGroup(
      //   new ToggleIntake(intake),
      //   new DriveToDistance(drive, 0.5)
      // ),
      // new ParallelCommandGroup(
      //   new ToggleIntake(intake),
      //   new DriveToDistance(drive, 0.5)
      // ),
      // //Standing Turn
      // new ManualShoot(shooter, 50, 50),
      // Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-2"), drive, true),
      // new ParallelCommandGroup(
      //   new ToggleIntake(intake),
      //   new DriveToDistance(drive, 0.5)
      // ),
      // new ParallelCommandGroup(
      //   new ToggleIntake(intake),
      //   new DriveToDistance(drive, 0.5)
      // ),
      // //Standing Turn
      // Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-3"), drive, true),
      // new ManualShoot(shooter, 50, 50)
      new Teleport(drive, new Pose2d(8.27, 2.714, new Rotation2d(1.27382855))),

      new DriveToDistance(drive, -Constants.Autonomous.kHubToBall/2),
      new TurnToAngle(180, drive),
      new DriveToDistance(drive, Constants.Autonomous.kHubToBall/2 - Constants.Autonomous.kTolerance),
      new ParallelRaceGroup(
        new DriveToDistance(drive, Constants.Autonomous.kTolerance),
        new IntakeRun(intake)
      ),
      new TurnToAngle(170, drive),
      new DriveToDistance(drive, Constants.Autonomous.kBallToShoot + Constants.Autonomous.kTolerance),

      new ParallelDeadlineGroup(
        new WaitCommand(Constants.Autonomous.kShootDuration),
        new ManualShoot(shooter, 50, 50)
      ),

      //Ok, we're done shooting and we're back at our original position
      new TurnToAngle(-70, drive),
      new DriveToDistance(drive, 2.05),

      new DriveToDistance(drive, Constants.Autonomous.kBallToShoot + Constants.Autonomous.kTolerance),
      
      new TurnToAngle(120, drive),
      new DriveToDistance(drive, Constants.Autonomous.kBallToShoot + Constants.Autonomous.kTolerance),

      new ParallelDeadlineGroup(
        new WaitCommand(Constants.Autonomous.kShootDuration),
        new ManualShoot(shooter, 50, 50)
      )

    );

  }
}