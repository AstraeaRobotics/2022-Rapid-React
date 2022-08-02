// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBall extends SequentialCommandGroup {
  /** Creates a new FourBall. */

  public FourBall(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(drive, Traj.createNewTrajectoryFromJSON("FourBall-1")),
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("FourBall-1"), drive, true),
      //Intake
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("FourBall-2"), drive, true),
      //Shoot
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("FourBall-3"), drive, true),
      //Intake
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("FourBall-4"), drive, true),
      //Intake
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("FourBall-5"), drive, true)
      //Shoot
    );
  }
}
