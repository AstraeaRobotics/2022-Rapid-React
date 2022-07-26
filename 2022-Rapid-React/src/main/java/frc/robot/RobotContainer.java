// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DriveWait;
import frc.robot.commands.SimDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

  /* GAMEPADS */
  private static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.kDriverControllerPort);
  private static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.kOperatorControllerPort);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

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

  public Command getAutonomousCommand() {
    // RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("OneBall-1").getInitialPose());

    // return new SequentialCommandGroup(
    //     Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("OneBall-1"), m_driveSubsystem, true));

    // RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("TwoBall").getInitialPose());
    // return new SequentialCommandGroup(
    //   Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("TwoBall"), m_driveSubsystem, true)
    // );

    // RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("CirclePath").getInitialPose());
    // return new SequentialCommandGroup(
    //   Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("CirclePath"), m_driveSubsystem, true));

    /* //ThreeBall
    RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("ThreeBall-1").getInitialPose());
    return new SequentialCommandGroup(
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-1"), m_driveSubsystem, true)
      //Intake
      //Standing Turn
      //Shoot
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-2"), m_driveSubsystem, true)
      //Intake
      //Standing Turn
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("ThreeBall-3"), m_driveSubsystem, true)
      //Shoot
    );
    */

    RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("TwoBall-1").getInitialPose());
    return new SequentialCommandGroup(
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("TwoBall-1"), m_driveSubsystem, true),
      //Intake
      Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("TwoBall-2"), m_driveSubsystem, true)
      //Shoot
      );

    // RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("StraightPath").getInitialPose());
    //   return new SequentialCommandGroup(
    //     Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("StraightPath"), m_driveSubsystem, true));
    // // return new DriveWait(m_driveSubsystem);
  }

}
