// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double distance = SmartDashboard.getNumber("AutoDistance", 0);

  public StraightPath(DriveSubsystem drive) {
  // SmartDashboard.putNumber("AUTO - Straight Path Speed", 10);
    System.out.println("Distance: " + SmartDashboard.getNumber("AutoDistance", 2));
    addCommands(
      new DriveToDistance(drive, distance)
    );
  }
}
