// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
public class FourBall extends SequentialCommandGroup {
  /** Creates a new FourBall. */

  public FourBall(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Teleport(drive, new Pose2d(8.27, 2.714, new Rotation2d(1.27382855+3.1415926535))),

      new DriveToDistance(drive, Constants.Autonomous.kHubToBall),
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

      new ParallelDeadlineGroup(
        Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("4Ball V1.0 Part2"), drive, true),
        new IntakeRun(intake)
      ),

      new WaitCommand(1),

      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("4Ball V1.0 Part2"), drive, true),

      new ParallelDeadlineGroup(
        new WaitCommand(Constants.Autonomous.kShootDuration),
        new ManualShoot(shooter, 50, 50)
      )
    );
  }
}
