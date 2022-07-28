// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightPath extends SequentialCommandGroup {
  /** Creates a new StraightPath. */
  // public static final Traj kInitialTrajectory = Traj.createNewTrajectoryFromJSON("StraightPath");

  DriveSubsystem m_driveSubsystem;

  public StraightPath(DriveSubsystem drive) {
    addCommands(
      new ResetOdometry(drive, Traj.createNewTrajectoryFromJSON("StraightPath")),
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("StraightPath"), drive, true)
    );
  }
}
