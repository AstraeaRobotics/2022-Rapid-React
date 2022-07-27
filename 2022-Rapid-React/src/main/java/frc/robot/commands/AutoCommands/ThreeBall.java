// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBall extends SequentialCommandGroup {
  /** Creates a new ThreeBall. */
  public static final Traj kInitialTrajectory = Traj.createNewTrajectoryFromJSON("ThreeBall-1");

  public ThreeBall(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-1"), drive, true),
      //Intake
      //Standing Turn
      //Shoot
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-2"), drive, true),
      //Intake
      //Standing Turn
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-3"), drive, true)
      //Shoot
    );

  }
}
