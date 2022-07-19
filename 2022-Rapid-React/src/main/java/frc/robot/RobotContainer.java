// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SimDrive;
import frc.robot.commands.TurretCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;
import frc.robot.util.Traj;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.TurretSubsystem;
//import frc.robot.commands.TurretCommand;

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
  public static final PS4Controller driverGamepad = new PS4Controller(Constants.RobotMap.DRIVER_CONTROLLER_PORT);
  public static final PS4Controller operatorGamepad = new PS4Controller(Constants.RobotMap.OPERATOR_CONTROLLER_PORT);

  // Right
  private final JoystickButton circleButton = new JoystickButton(driverGamepad, 3);
  private final JoystickButton squareButton = new JoystickButton(driverGamepad, 1);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  public TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new SimDrive());
  }

  private void configureButtonBindings() {
    squareButton.whileHeld(new TurretCommand(m_turretSubsystem, -0.05));
    circleButton.whileHeld(new TurretCommand(m_turretSubsystem, 0.05));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    RobotContainer.m_driveSubsystem.resetOdometry(Traj.createNewTrajectoryFromJSON("OneBall-1").getInitialPose());

    return new SequentialCommandGroup(
        Ramsete.createRamseteCommand(Traj.createNewTrajectoryFromJSON("OneBall-1"), m_driveSubsystem, true));
  }
}