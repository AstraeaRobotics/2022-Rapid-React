// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveToDistance;
import frc.robot.commands.SimDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* GAMEPADS */
  private static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.kDriverControllerPort);
  private static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.kOperatorControllerPort);

  private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(
        new SimDrive(m_driveSubsystem, 2,
            driverGamepad::getR2Axis,
            driverGamepad::getL2Axis,
            driverGamepad::getLeftX,
            driverGamepad::getRightX));
  }

  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("OneBall-1").getInitialPose());

    // return new SequentialCommandGroup(
    //     Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("OneBall-1"), m_driveSubsystem, true));

    return new DriveToDistance(m_driveSubsystem, 2.0);
  }
}
